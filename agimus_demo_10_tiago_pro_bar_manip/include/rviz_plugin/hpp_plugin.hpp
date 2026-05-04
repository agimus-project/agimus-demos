#pragma once

#include <QComboBox>
#include <QDateTime>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>

#include "test_agimus_type/action/plan_bar_grasp.hpp"

namespace hpp_plugin {

class OrchestratorPannel : public rviz_common::Panel {
  // NO Q_OBJECT - avoids MOC vtable issues with pluginlib on RViz2/Humble

 public:
  using PlanBarGrasp = test_agimus_type::action::PlanBarGrasp;
  using GoalHandlePBG = rclcpp_action::ClientGoalHandle<PlanBarGrasp>;

  explicit OrchestratorPannel(QWidget* parent = nullptr);
  ~OrchestratorPannel() override;

  void onInitialize() override;

 private:
  // == UI ==================================================================
  QComboBox* action_type_combo_;
  QComboBox* gripper_combo_;
  QComboBox* handle_combo_;
  QPushButton* send_btn_;
  QPushButton* cancel_btn_;
  QTextEdit* log_view_;

  // == ROS =================================================================
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<PlanBarGrasp>::SharedPtr action_client_;
  GoalHandlePBG::SharedPtr goal_handle_;

  // == helpers =============================================================
  void sendGoal();
  void cancelGoal();
  void log(const QString& msg, const QString& color = "white");
  void setUiBusy(bool busy);

  void goalResponseCallback(GoalHandlePBG::SharedPtr handle);
  void feedbackCallback(GoalHandlePBG::SharedPtr,
                        std::shared_ptr<const PlanBarGrasp::Feedback> feedback);
  void resultCallback(const GoalHandlePBG::WrappedResult& result);
};

}  // namespace hpp_plugin
