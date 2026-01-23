from aligator_mpc import MPC, Config, PatternGenerator, TestTrajs
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from linear_feedback_controller_msgs.msg import Control, Sensor
from sensor_msgs.msg import JointState
from std_msgs.msg import MultiArrayDimension, ColorRGBA, Float32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.duration import Duration
import pinocchio as pin
from pathlib import Path
import numpy as np
import sys
import aligator

# !! ===debug===
import time
from line_profiler import profile


class AligatorMPC(Node):
    def __init__(self):
        super().__init__('aligator_mpc')

        # MPC configuration ========================================================
        self.get_logger().info('Aligator MPC starting...')
        self.declare_parameter('config',"clearly a wrong value!!")
        config_path = self.get_parameter('config').value

        self.get_logger().info(config_path)
        self.mpc_parameters = Config.from_yaml(Path(config_path))

        # Waypoints ================================================================
        patternGen = PatternGenerator([0.3,0.3,0], (0.5, 0,0.1))
        mpc_waypoints = patternGen.generate_pattern('zigzag',stride=0.05)
        
        # test_trajs = TestTrajs()
        # start = pin.SE3(pin.rpy.rpyToMatrix(np.pi,0,0), np.array([0.5, -0.2, 0.2]))
        # end =  pin.SE3(pin.rpy.rpyToMatrix(np.pi,0,np.pi/2), np.array([0.5, 0.2, 0.2]))
        # mpc_waypoints = [start, end]

        # startsin = [0.3, -0., 0.2]
        # mpc_waypoints = test_trajs.sine(start_point=startsin,length=0.3,period=0.03,amplitude=0.15, dist_between_points=0.01, sine_axis="Y", ampl_axis="X")

        self.waypoints_marker_msg = Marker() #self.waypoints_to_marker(mpc_waypoints)
        
        # Solver setup ============================================================
        self.robot_state = None
        self.mpc = MPC(mpc_waypoints, self.mpc_parameters)
        self.first_mpc_iteration = True
        self.feedback_gain = 1e-4

        # ROS2 publishers & subscribers ============================================
        self.control_publisher = self.create_publisher(Control, "control", 10)
        self.mpc_pred_publisher = self.create_publisher(Marker, "mpc_prediction_endeffector",10)
        self.mpc_waypoints_publisher = self.create_publisher(Marker, "mpc_waypoints", 10)
        self.mpc_time_publisher = self.create_publisher(Float32, "mpc_time",10)
        self.mpc_warmstart_publisher = self.create_publisher(JointState, "mpc_warmstart", 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(Sensor, 'sensor', self.sensor_callback, qos)
        self.launch_srv = self.create_service(Trigger, 'aligator_mpc/launch_mpc', self.launch_mpc_callback)
        self.mpc_started = False

        # Publisher timers ===============================================================
        ctrl_timer_period = self.mpc_parameters.mpc.dt
        ctrl_timer = self.create_timer(ctrl_timer_period, self.publish_control_callback)
        waypoints_timer_period = 0.1
        waypoints_timer = self.create_timer(waypoints_timer_period,self.publish_waypoints_callback)

        self.get_logger().info('End of Setup')

    # @profile
    def publish_control_callback(self):
            start = time.time()
            if self.mpc_started:
                
                time_mpc = self.mpc.iterate(self.robot_state, self.last_effort_meas)
                # self.get_logger().info(f'mpc time: {(time_mpc)}')
                
                command = [v for v in self.mpc.results.us.tolist()[0]]
                # command = command[:-2]
                
                raw_feedback_gains = self.mpc.results.controlFeedbacks().tolist()[0]
                feedback_gains = self.flatten_feedback_gains(raw_feedback_gains)
                # self.get_logger().info(f'feddb gains: {(feedback_gains)}')

                #! ================== debug  predicted ee traj ==============
                if True:
                    xs_array = self.mpc.results.xs.tolist()
                    marker_array_msg = self.xs_to_marker(xs_array)
                    self.mpc_pred_publisher.publish(marker_array_msg)

                control_msg = Control()
                dim_rows_ffw = MultiArrayDimension()
                dim_rows_ffw.label = "rows"
                dim_rows_ffw.size = 7
                dim_rows_ffw.stride = 7
                control_msg.feedforward.layout.dim.append(dim_rows_ffw)
                dim_cols_ffw = MultiArrayDimension()
                dim_cols_ffw.label = "cols"
                dim_cols_ffw.size = 1
                dim_cols_ffw.stride = 1
                control_msg.feedforward.layout.dim.append(dim_cols_ffw)
                control_msg.feedforward.data = [i*1 for i in command] 

                dim_rows_fb = MultiArrayDimension()
                dim_rows_fb.label = "rows"
                dim_rows_fb.size = 7
                dim_rows_fb.stride = 98
                control_msg.feedback_gain.layout.dim.append(dim_rows_fb)
                dim_cols_fb = MultiArrayDimension()
                dim_cols_fb.label = "cols"
                dim_cols_fb.size = 14
                dim_cols_fb.stride = 14
                control_msg.feedback_gain.layout.dim.append(dim_cols_fb)
                #!!================================= 
                id_gains = np.identity(7)
                id_gains = np.hstack((id_gains, id_gains))
                id_gains = id_gains.flatten().tolist()
                control_msg.feedback_gain.data =   [i*self.feedback_gain for i in feedback_gains] 
                # control_msg.feedback_gain.data = id_gains

                sensor_msg = Sensor()
                x_desired = [float(val) for val in self.mpc.results.xs.tolist()[0]]
                joint_state_msg = JointState()
                joint_state_msg.name = self.last_sensor_msg.joint_state.name
                joint_state_msg.position = x_desired[:self.mpc.n_q]
                joint_state_msg.velocity = x_desired[self.mpc.n_q:]
                sensor_msg.joint_state = joint_state_msg

                control_msg.initial_state =  sensor_msg # self.last_sensor_msg #

                sensor_x1_msg = Sensor()
                x_desired = [float(val) for val in self.mpc.results.xs.tolist()[1]]
                joint_state_x1_msg = JointState()
                joint_state_x1_msg.name = self.last_sensor_msg.joint_state.name
                joint_state_x1_msg.position = x_desired[:self.mpc.n_q]
                joint_state_x1_msg.velocity = x_desired[self.mpc.n_q:]
                sensor_x1_msg.joint_state = joint_state_x1_msg

                control_msg.initial_state_x1 = sensor_x1_msg # sensor_msg #

                self.control_publisher.publish(control_msg)
                stop = time.time()
                delta_t = Float32()
                delta_t.data = stop - start
                if (stop - start) > self.mpc_parameters.mpc.dt:
                    self.get_logger().warning(f"MPC OVERRUN ({stop-start})")
                self.mpc_time_publisher.publish(delta_t)


    def publish_waypoints_callback(self):
        self.mpc_waypoints_publisher.publish(self.waypoints_marker_msg)

    def sensor_callback(self, msg):
        self.last_sensor_msg = msg

        # Get robot state =========================================================
        position = list(msg.joint_state.position)
        # position = np.array(position  + [0.,0.])
        position = np.array(position)


        velocity = list(msg.joint_state.velocity)
        # velocity = np.array(velocity  + [0.,0.])
        velocity = np.array(velocity)

        self.last_effort_meas = list(msg.joint_state.effort)
        # self.last_effort_meas = np.array(self.last_effort_meas + [0.,0.])
        self.last_effort_meas = np.array(self.last_effort_meas)

        self.robot_state = np.concatenate((position, velocity))
        self.mpc.setStartPose(position) # update pinocchio model
        if self.first_mpc_iteration:
            self.first_mpc_iteration = False

            self.mpc.initStages() # once the start pose is set the stages must be computed before the first solver iteration
            mpc_traj = self.mpc.stage_factory.getFullTrajectory_pt_by_pt()
            # self.get_logger().info(f' mpc traj : {mpc_traj}')
            self.waypoints_marker_msg = self.waypoints_to_marker(mpc_traj)
                    

        # TODO check joints limits

        
    def launch_mpc_callback(self, request, response):
        self.mpc_started = True
        response.success = True # TODO : add checks to see if ready to launch
        response.message = "MPC launched"
        self.get_logger().info('Aligator MPC launched')

        return response

    def xs_to_marker(self, xs_table):
        """Converts a list of xs values to a line marker following the end effector of the robot

        Args:
            xs_table (list[list[float]]): list of robot states

        Returns:
            marker Marker: Line marker message
        """

        points_list = []
        color_list = []

        marker = Marker()
        marker.header.frame_id = "fer_link0";
        marker.type = Marker.LINE_STRIP;
        marker.scale.x = 0.005;
        nb_points = len(xs_table)
        color_gradient = np.linspace(0.0, 1.0, nb_points)

        id = 0
        for xs in xs_table:
            qs = xs[:self.mpc.n_q]
            endeffector_pose = self.mpc.get_endpoint(qs)

            point = Point()
            point.x = endeffector_pose[0]
            point.y = endeffector_pose[1]
            point.z = endeffector_pose[2]
            color = ColorRGBA()
            color.r = color_gradient[id] 
            color.g = 0.
            color.b = color_gradient[-id]
            color.a = 1.0

            points_list.append(point)
            color_list.append(color)
            id += 1

        marker.points = points_list
        marker.colors = color_list
        return marker

    def flatten_feedback_gains(self,raw_feedback_gains):
        """Converts the feedback gains from the aligator formatting (2D table) to the /Control message format (1D formatting for a MultiArrayMatrix ros message). Also removes the gains for the gripper (last 2 links).

        Args:
            raw_feedback_gains (list[list[float]]): raw feedback gains from the aligator solver
        Return:
            feedback_gain (list[float]): flattenend feedback gains matrix
        """
        # flatten the gains from a (rows, cols) 2D format to flat list of [row1 row2 row3]
        feedback_gains = []
        for gains in raw_feedback_gains:
            # self.get_logger().info(f'feddb gains: {(gains)}')
            gains = [float(k)for k in gains] # convert values to float 
            # self.get_logger().info(f'feddb gains: {type(gains)}')
            # gains = gains[:self.mpc.n_q] + gains[self.mpc.n_q:] # remove gripper gains on q and on vs

            feedback_gains = feedback_gains + gains # add gains list at the end of the master list

        return feedback_gains

    def waypoints_to_marker(self, waypoints):

        points_list = []
        marker = Marker()
        marker.header.frame_id = "fer_link0";
        marker.type = Marker.LINE_STRIP;
        marker.scale.x = 0.01;
        marker.lifetime = Duration().to_msg()
        color = ColorRGBA()
        color.r = 0.5
        color.g = 1.
        color.b = 0.
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

