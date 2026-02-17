from aligator_mpc import MPC, Config, PatternGenerator
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from linear_feedback_controller_msgs.msg import Control, Sensor
from sensor_msgs.msg import JointState
from std_msgs.msg import MultiArrayDimension, ColorRGBA, Float32, String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.duration import Duration
from pathlib import Path
import numpy as np
from copy import deepcopy
import time


class AligatorMPC(Node):
    def __init__(self) -> None:
        super().__init__("aligator_mpc")

        # MPC configuration ========================================================
        self.get_logger().info("Aligator MPC starting...")
        self.declare_parameter("config", "No path set for mpc config file !")
        config_path = self.get_parameter("config").value
        self.mpc_parameters = Config.from_yaml(Path(config_path))

        # Waypoints ================================================================
        patternGen = PatternGenerator([0.36, 0.5, 0], (0.52, 0.0, 0.087))  # testing
        # patternGen = PatternGenerator([0.24,0.3,0], (0.5, 0,0.3)) # real box

        self.mpc_waypoints = patternGen.generate_pattern("zigzag_curve", stride=0.05)

        # test_trajs = TestTrajs()
        # start = pin.SE3(pin.rpy.rpyToMatrix(np.pi,0,0), np.array([0.5, -0.2, 0.2]))
        # end =  pin.SE3(pin.rpy.rpyToMatrix(np.pi,0,np.pi/2), np.array([0.5, 0.2, 0.2]))
        # mpc_waypoints = [start, end]

        # startsin = [0.3, -0., 0.2]
        # mpc_waypoints = test_trajs.sine(start_point=startsin,length=0.3,period=0.03,amplitude=0.15, dist_between_points=0.01, sine_axis="Y", ampl_axis="X")

        self.waypoints_marker_msg = Marker()

        # Solver setup ============================================================
        self.robot_state = None
        self.mpc = None
        self.mpc_ready = False
        self.first_mpc_iteration = True
        self.feedback_gain_scaling = 1.0

        # ROS2 publishers & subscribers ============================================
        self.control_publisher = self.create_publisher(Control, "control", 10)
        self.base_control_msg = self.build_base_control_msg()
        self.mpc_ee_pred_published = self.create_publisher(
            Marker, "aligator_mpc/ee_prediction_on_horizon", 10
        )
        self.mpc_imput_waypoints_publisher = self.create_publisher(
            Marker, "aligator_mpc/input_waypoints", 10
        )
        self.mpc_time_publisher = self.create_publisher(
            Float32, "aligator_mpc/pub_control_duration", 10
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_r_desc = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.subscription_sensor = self.create_subscription(
            Sensor, "sensor", self.sensor_callback, qos
        )
        self.subscription_robot_descr = self.create_subscription(
            String, "robot_description", self.robot_descr_callback, qos_r_desc
        )
        self.launch_srv = self.create_service(
            Trigger, "aligator_mpc/launch_mpc", self.launch_mpc_callback
        )
        self.mpc_started = False

        # Publisher timers ===============================================================
        ctrl_timer_period = self.mpc_parameters.mpc.dt
        _ = self.create_timer(ctrl_timer_period, self.publish_control_callback)
        waypoints_timer_period = 0.1
        _ = self.create_timer(waypoints_timer_period, self.publish_waypoints_callback)

        self.get_logger().info("End of Setup")

    def publish_control_callback(self) -> None:
        """Iterates the MPC
        \n Publishes the feedforward and feedback gains on topic `/Control`
        \n Publishes the end effector predicted pose on topic `/aligator_mpc/ee_prediction_on_horizon`
        \n Publishes the time it took to iterate and publish on topic `aligator_mpc/pub_control_duration`
        """

        start = time.time()
        if self.mpc_started:
            _ = self.mpc.iterate(
                self.robot_state
            )  # outputs the time it took to iterate which is currently unused

            feedback_gains = (
                -self.feedback_gain_scaling * self.mpc.results.controlFeedbacks()[0]
            )

            # show end effector position during predicted horizon
            if True:
                xs_array = self.mpc.results.xs.tolist()
                marker_array_msg = self.xs_to_marker(xs_array)
                self.mpc_ee_pred_published.publish(marker_array_msg)

            # fill the non static values
            control_msg = deepcopy(self.base_control_msg)
            control_msg.feedforward.data = self.mpc.results.us[0].tolist()
            control_msg.feedback_gain.data = feedback_gains.flatten().tolist()

            sensor_msg = Sensor()
            x_desired = [float(val) for val in self.mpc.results.xs[0]]
            joint_state_msg = JointState()
            joint_state_msg.name = self.last_sensor_msg.joint_state.name
            joint_state_msg.position = x_desired[: self.mpc.nq].copy()
            joint_state_msg.velocity = x_desired[self.mpc.nq :].copy()
            sensor_msg.joint_state = joint_state_msg

            control_msg.initial_state = self.last_sensor_msg  # sensor_msg #

            self.control_publisher.publish(control_msg)
            stop = time.time()
            delta_t = Float32()
            delta_t.data = stop - start
            if (stop - start) > self.mpc_parameters.mpc.dt:
                self.get_logger().warning(f"MPC OVERRUN ({stop - start})")
            self.mpc_time_publisher.publish(delta_t)

    def publish_waypoints_callback(self) -> None:
        self.mpc_imput_waypoints_publisher.publish(self.waypoints_marker_msg)

    def sensor_callback(self, msg: Sensor) -> None:
        """Callback when a message is published on `/Sensor` topic. Updates the `self.robot_state` with the message data.
        \nIf the MPC is launched it will also call `mpc.initStages` and updates the `self.waypoints_marker_msg`

        Args:
            msg (Sensor)
        """

        self.last_sensor_msg = msg

        # Get robot state
        position = list(msg.joint_state.position)
        position = np.array(position)

        velocity = list(msg.joint_state.velocity)
        velocity = np.array(velocity)

        self.robot_state = np.concatenate((position, velocity))
        self.mpc.setStartPose(position)  # update pinocchio model in the MPC

        if self.first_mpc_iteration and self.mpc_ready:
            self.get_logger().info("MPC launched")
            self.first_mpc_iteration = False

            self.mpc.initStages()  # once the start pose is set the stages must be computed before the first solver iteration
            mpc_traj = self.mpc.stage_factory.getFullTrajectory_pt_by_pt()
            self.waypoints_marker_msg = self.waypoints_to_marker(mpc_traj)

    def robot_descr_callback(self, msg: String) -> None:
        """Gets the robot description from the /robot_description topic and starts the MPC

        Args:
            msg (String): topic message
        """

        robot_urdf = msg.data
        self.mpc = MPC(self.mpc_waypoints, self.mpc_parameters, robot_urdf)
        self.mpc_ready = True

    def launch_mpc_callback(self, request, response):
        """Callback triggered with service /mpc_launch is called

        Args:
            request : service request
            response : service response

        Returns:
            response: service response
        """
        self.mpc_started = True
        response.success = True
        response.message = "MPC launched"
        self.get_logger().info("Aligator MPC launched")

        return response

    # Utils functions
    def build_base_control_msg(self) -> Control:
        """Builts a control message prefilled with static values

        Returns:
            linear_feedback_controller_msgs.msg.Control: deepcopy of the prefilled control message
        """
        base_control_msg = Control()
        dim_rows_ffw = MultiArrayDimension()
        dim_rows_ffw.label = "rows"
        dim_rows_ffw.size = 7
        dim_rows_ffw.stride = 7
        base_control_msg.feedforward.layout.dim.append(dim_rows_ffw)
        dim_cols_ffw = MultiArrayDimension()
        dim_cols_ffw.label = "cols"
        dim_cols_ffw.size = 1
        dim_cols_ffw.stride = 1
        base_control_msg.feedforward.layout.dim.append(dim_cols_ffw)
        dim_rows_fb = MultiArrayDimension()
        dim_rows_fb.label = "rows"
        dim_rows_fb.size = 7
        dim_rows_fb.stride = 98
        base_control_msg.feedback_gain.layout.dim.append(dim_rows_fb)
        dim_cols_fb = MultiArrayDimension()
        dim_cols_fb.label = "cols"
        dim_cols_fb.size = 14
        dim_cols_fb.stride = 14
        base_control_msg.feedback_gain.layout.dim.append(dim_cols_fb)

        return base_control_msg

    def xs_to_marker(self, xs_table) -> Marker:
        """Converts a list of xs values to a line marker following the end effector of the robot

        Args:
            xs_table (list[list[float]]): list of robot states

        Returns:
            marker Marker: Line marker message
        """

        points_list = []
        color_list = []

        marker = Marker()
        marker.header.frame_id = self.mpc_parameters.robot.name + "_link0"
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.005
        nb_points = len(xs_table)
        color_gradient = np.linspace(0.0, 1.0, nb_points)

        id = 0
        for xs in xs_table:
            qs = xs[: self.mpc.nq]
            endeffector_pose = self.mpc.get_endpoint(qs)

            point = Point()
            point.x = endeffector_pose[0]
            point.y = endeffector_pose[1]
            point.z = endeffector_pose[2]
            color = ColorRGBA()
            color.r = color_gradient[id]
            color.g = 0.0
            color.b = color_gradient[-id]
            color.a = 1.0

            points_list.append(point)
            color_list.append(color)
            id += 1

        marker.points = points_list
        marker.colors = color_list
        return marker

    def waypoints_to_marker(self, waypoints: list) -> Marker:
        """Convert waypoint list to marker message

        Args:
            waypoints (list): waypoints list

        Returns:
            Marker: marker message
        """

        points_list = []
        marker = Marker()
        marker.header.frame_id = self.mpc_parameters.robot.name + "_link0"
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.01
        marker.lifetime = Duration().to_msg()
        color = ColorRGBA()
        color.r = 0.5
        color.g = 1.0
        color.b = 0.0
        color.a = 0.3

        for waypoint in waypoints:
            point = Point()
            point.x = waypoint[0]
            point.y = waypoint[1]
            point.z = waypoint[2]
            points_list.append(point)

        marker.points = points_list
        marker.color = color

        return marker


def main(args=None):
    rclpy.init(args=args)

    node = AligatorMPC()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
