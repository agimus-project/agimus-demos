#include "agimus_demos_controllers/ft_calibration_filter.hpp"

#include <Eigen/Dense>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/skew.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <std_msgs/msg/string.hpp>

#include "controller_interface/helpers.hpp"

namespace agimus_demos_controllers {

controller_interface::CallbackReturn FTCalibrationFilter::on_init() {
  try {
    param_listener_ =
        std::make_shared<ft_calibration_filter::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FTCalibrationFilter::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = params_.filtered_forces_interfaces_names;

  if (params_.contact_detection.augment_state) {
    command_interfaces_config.names.push_back(
        params_.contact_detection.command_interface_name);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
FTCalibrationFilter::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::ALL};
}

controller_interface::CallbackReturn FTCalibrationFilter::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  try {
    // register ft sensor data publisher
    sensor_state_publisher_ =
        get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "~/wrench", rclcpp::SystemDefaultsQoS());
    realtime_wrench_publisher_ =
        std::make_unique<StatePublisher>(sensor_state_publisher_);
  } catch (const std::exception& e) {
    fprintf(stderr,
            "Exception thrown during publisher creation at configure stage "
            "with message : %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  realtime_wrench_publisher_->lock();
  realtime_wrench_publisher_->msg_.header.frame_id =
      params_.measurement_frame_id;
  realtime_wrench_publisher_->unlock();

  try {
    // register ft sensor data publisher
    contact_publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
        "~/contact", rclcpp::SystemDefaultsQoS());
    realtime_contact_publisher_ =
        std::make_unique<ContactPublisher>(contact_publisher_);
  } catch (const std::exception& e) {
    fprintf(stderr,
            "Exception thrown during publisher creation at configure stage "
            "with message : %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  realtime_contact_publisher_->lock();
  realtime_contact_publisher_->msg_.data = false;
  realtime_contact_publisher_->unlock();

  calibrate_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/calibrate", std::bind(&FTCalibrationFilter::calibrate_sensor_cb, this,
                               std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_node()->get_logger(), "configure successful");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FTCalibrationFilter::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  params_ = param_listener_->get_params();

  if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, params_.filtered_forces_interfaces_names,
          std::string(""), ordered_command_interfaces_)) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Expected %zu command interfaces, found %zu",
                 params_.filtered_forces_interfaces_names.size(),
                 ordered_command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, params_.state_force_interfaces_names,
          std::string(""), ordered_state_force_interfaces_)) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Expected %zu force sensor state interfaces, found %zu",
                 params_.state_force_interfaces_names.size(),
                 ordered_state_force_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, params_.state_robot_position_interfaces_names,
          std::string(""), ordered_state_robot_position_interfaces_)) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Expected %zu robot position state interfaces, found %zu",
                 params_.state_robot_position_interfaces_names.size(),
                 ordered_state_robot_position_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.contact_detection.augment_state) {
    bool found = false;
    const auto& name = params_.contact_detection.command_interface_name;
    for (auto& interface : command_interfaces_) {
      if (name == interface.get_name()) {
        ordered_command_interfaces_.push_back(std::ref(interface));
        found = true;
        break;
      }
    }
    if (!found) {
      RCLCPP_ERROR(
          this->get_node()->get_logger(),
          "New command interface for contact detection was not found!");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  std_msgs::msg::String robot_description_msg;
  // Create temporary ROS node to be able to spin it
  auto robot_description_node = std::make_shared<rclcpp::Node>(
      get_node()->get_name() + std::string("_robot_description"));
  auto robot_description_sub =
      robot_description_node->create_subscription<std_msgs::msg::String>(
          "/robot_description", rclcpp::QoS(1).transient_local(),
          [](const std::shared_ptr<const std_msgs::msg::String>) {});

  const std::size_t retires = 10;
  for (std::size_t i = 0; i < retires; i++) {
    rclcpp::spin_some(robot_description_node);
    if (rclcpp::wait_for_message(robot_description_msg, robot_description_sub,
                                 get_node()->get_node_options().context(),
                                 std::chrono::seconds(1))) {
      RCLCPP_INFO(get_node()->get_logger(), "Robot description received.");
      break;
    } else if (i == retires) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Filed to receive data on '/robot_description' topic.");
      return controller_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_node()->get_logger(),
                "Robot description still not received. Retrying...");
  }

  pinocchio::Model robot_model_full;
  pinocchio::urdf::buildModelFromXML(robot_description_msg.data,
                                     robot_model_full);
  std::vector<pinocchio::JointIndex> moving_joint_ids;
  for (const std::string joint_name : params_.moving_joint_names) {
    if (robot_model_full.existJointName(joint_name)) {
      const auto joint_id = robot_model_full.getJointId(joint_name);
      moving_joint_ids.push_back(joint_id);
    } else {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Filed to find joint with a name '%s' in the URDF!",
                   joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  std::vector<pinocchio::JointIndex> locked_joint_ids;
  for (std::size_t i = 1; i < robot_model_full.njoints; i++) {
    if (std::find(moving_joint_ids.begin(), moving_joint_ids.end(), i) ==
        moving_joint_ids.end()) {
      locked_joint_ids.push_back(i);
    }
  }

  Eigen::VectorXd q_default_complete =
      Eigen::VectorXd::Zero(robot_model_full.nq);
  robot_model_ = pinocchio::buildReducedModel(
      robot_model_full, locked_joint_ids, q_default_complete);
  robot_data_ = pinocchio::Data(robot_model_);

  // Preallocate matrix storing bias values for averaging
  bias_measurements_ = Matrix6Xd(6, params_.bias_measurement_samples);
  // Preallocate state vector
  q_ = Eigen::VectorXd::Zero(robot_model_.nq);

  g_ = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(
      params_.gravity_vector.data(), params_.gravity_vector.size());

  auto& calib = params_.calibration.measurement_frame;
  Eigen::Vector3d rpy = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(
      calib.rpy.data(), calib.rpy.size());
  Eigen::Vector3d xyz = Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(
      calib.xyz.data(), calib.xyz.size());
  calibration_ = pinocchio::SE3(pinocchio::rpy::rpyToMatrix(rpy), xyz);
  calibration_trans_.template topLeftCorner<3, 3>() =
      Eigen::Matrix3d::Identity();
  calibration_trans_.template bottomLeftCorner<3, 3>() = pinocchio::skew(xyz);

  const auto& frame_name = params_.measurement_frame_id;
  if (!robot_model_.existFrame(frame_name)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Filed to find find with a name '%s' in the URDF!",
                 frame_name.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  frame_of_interest_id_ = robot_model_.getFrameId(frame_name);

  bias_buffer_cnt_ = 0;
  bias_computed_ = false;
  avg_bias_ = pinocchio::Force::Zero();

  for (std::size_t i = 0; i < 6; i++) {
    const auto& filter_params = params_.state_force_interfaces_names_map.at(
        params_.state_force_interfaces_names[i]);

    std::array<double, 2> a;
    std::array<double, 3> b;
    std::copy_n(filter_params.a.begin(), 2, a.begin());
    std::copy_n(filter_params.b.begin(), 3, b.begin());
    filters_.push_back(ButterworthFilter(a, b));
  }

  contact_detector_.set_hysteresis_samples(
      params_.contact_detection.hysteresis_samples);
  contact_detector_.set_lower_threshold(
      params_.contact_detection.lower_threshold);
  contact_detector_.set_upper_threshold(
      params_.contact_detection.upper_threshold);
  contact_detector_.set_axis_mask(params_.contact_detection.axis_mask);

  RCLCPP_INFO(this->get_node()->get_logger(), "activate successful");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FTCalibrationFilter::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  ordered_state_force_interfaces_.clear();
  ordered_state_robot_position_interfaces_.clear();
  ordered_command_interfaces_.clear();
  release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

void FTCalibrationFilter::calibrate_sensor_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_INFO(this->get_node()->get_logger(), "Calibrating sensor bias.");

  auto logger = this->get_node()->get_logger();

  // Since in humble get_update_rate() doesn't work, we need a walkaround...
  const double update_rate = static_cast<double>(params_.update_rate);
  const double samples = static_cast<double>(params_.bias_measurement_samples);
  // Compute time expected to wait for the calibration to take
  // while accounting for some slack
  const double calibration_time_slack = 1.05;
  const double wait_time = samples / update_rate * calibration_time_slack;

  const auto start_time = std::chrono::steady_clock::now();
  const auto timeout_sec = std::chrono::duration<double>(wait_time);
  bool timeout = false;

  // Inform main loop that bias computation is expected
  bias_computed_ = false;
  // Wait bias to be computed
  while (!bias_computed_ && !timeout) {
    timeout = (std::chrono::steady_clock::now() - start_time) > timeout_sec;
  }

  response->success = !timeout;
  if (timeout) {
    const std::string msg = "Calibration failed! Timeout reached!";
    response->message = msg;
    RCLCPP_ERROR(this->get_node()->get_logger(), msg.c_str());
  } else {
    response->message = "Calibration succeeded.";
  }
}

controller_interface::return_type FTCalibrationFilter::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    contact_detector_.set_hysteresis_samples(
        params_.contact_detection.hysteresis_samples);
    contact_detector_.set_lower_threshold(
        params_.contact_detection.lower_threshold);
    contact_detector_.set_upper_threshold(
        params_.contact_detection.upper_threshold);
    contact_detector_.set_axis_mask(params_.contact_detection.axis_mask);
    RCLCPP_INFO(this->get_node()->get_logger(), "Parameters were updated");
  }

  Vector6d force;
  for (std::size_t i = 0; i < 6; i++) {
    force[i] = ordered_state_force_interfaces_[i].get().get_value();
  }
  force_.toVector() = params_.invert_forces ? -force : force;

  for (std::size_t i = 0; i < q_.size(); i++) {
    q_[i] = ordered_state_robot_position_interfaces_[i].get().get_value();
  }

  pinocchio::forwardKinematics(robot_model_, robot_data_, q_);
  pinocchio::SE3 T_frame = pinocchio::updateFramePlacement(
      robot_model_, robot_data_, frame_of_interest_id_);

  const double m = params_.calibration.com.mass;
  const auto f_gravity =
      calibration_trans_ * (m * calibration_.rotation().transpose() *
                            T_frame.rotation().transpose() * g_);

  // If all data required was acquired, compute average bias
  if (!bias_computed_) {
    // Gather measurements to later remove bias
    if (bias_buffer_cnt_ < bias_measurements_.cols()) {
      bias_measurements_.col(bias_buffer_cnt_) = force_.toVector() - f_gravity;
      bias_buffer_cnt_++;
      // Assume values are frozen during calibration
      return controller_interface::return_type::OK;
    }
    avg_bias_.toVector() = bias_measurements_.rowwise().mean();
    bias_computed_ = true;
    bias_buffer_cnt_ = 0;
    RCLCPP_INFO(this->get_node()->get_logger(),
                "Bias computation finished. Bias Values are:\n"
                "\tforce.x:  %3.2fN\n"
                "\tforce.y:  %3.2fN\n"
                "\tforce.z:  %3.2fN\n"
                "\ttorque.x: %3.2fNm\n"
                "\ttorque.y: %3.2fNm\n"
                "\ttorque.z: %3.2fNm",
                avg_bias_.linear()[0], avg_bias_.linear()[1],
                avg_bias_.linear()[2], avg_bias_.angular()[0],
                avg_bias_.angular()[1], avg_bias_.angular()[2]);
    return controller_interface::return_type::OK;
  }

  pinocchio::Force f_out(force_.toVector() - avg_bias_.toVector() - f_gravity);

  for (std::size_t i = 0; i < f_out.toVector().size(); i++) {
    f_out.toVector()[i] = filters_[i].update(f_out.toVector()[i]);
    ordered_command_interfaces_[i].get().set_value(f_out.toVector()[i]);
  }

  if (realtime_wrench_publisher_ && realtime_wrench_publisher_->trylock()) {
    realtime_wrench_publisher_->msg_.header.stamp = time;
    realtime_wrench_publisher_->msg_.wrench.force.x = f_out.linear()[0];
    realtime_wrench_publisher_->msg_.wrench.force.y = f_out.linear()[1];
    realtime_wrench_publisher_->msg_.wrench.force.z = f_out.linear()[2];
    realtime_wrench_publisher_->msg_.wrench.torque.x = f_out.angular()[0];
    realtime_wrench_publisher_->msg_.wrench.torque.y = f_out.angular()[1];
    realtime_wrench_publisher_->msg_.wrench.torque.z = f_out.angular()[2];
    realtime_wrench_publisher_->unlockAndPublish();
  }

  contact_detector_.update(f_out);
  bool in_contact = contact_detector_.in_contact();

  if (realtime_contact_publisher_ && realtime_contact_publisher_->trylock()) {
    realtime_contact_publisher_->msg_.data = in_contact;
    realtime_contact_publisher_->unlockAndPublish();
  }

  if (params_.contact_detection.augment_state) {
    ordered_command_interfaces_[ordered_command_interfaces_.size() - 1]
        .get()
        .set_value(static_cast<double>(in_contact));
  }

  return controller_interface::return_type::OK;
}

}  // namespace agimus_demos_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(agimus_demos_controllers::FTCalibrationFilter,
                       controller_interface::ControllerInterface)
