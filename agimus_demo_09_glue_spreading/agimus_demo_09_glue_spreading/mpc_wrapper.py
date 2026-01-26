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
        patternGen = PatternGenerator([0.24,0.3,0], (0.6, -0.16,0.1)) # testing
        # patternGen = PatternGenerator([0.24,0.3,0], (0.5, 0,0.3)) # real box

        mpc_waypoints = patternGen.generate_pattern('zigzag_curve',stride=0.035)
        
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
        self.feedback_gain_scaling = 5e-2
        self.x_desired = [
         -4.4941237144485114e-07
        ,-0.7808052627058467
        , 2.48589228672936e-15
        ,-2.4126768818223634
        ,-3.1345563144870245e-14
        ,1.5703322669120232
        ,0.7799999999999953
        ,0, 0, 0, 0, 0, 0, 0] 

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
                
                time_mpc = self.mpc.iterate(self.robot_state)
                # self.get_logger().info(f'mpc time: {(time_mpc)}')

                feedback_gains = self.feedback_gain_scaling * self.mpc.results.controlFeedbacks()[0]
                # feedback_gains[:, 7:14] = np.zeros((7,7))
              
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

                #### MON CONTROL EST ICI
                control_msg.feedforward.data = self.mpc.results.us[0].tolist()

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

                control_msg.feedback_gain.data =  feedback_gains.flatten().tolist()

                sensor_msg = Sensor()
                x_desired = [float(val) for val in self.mpc.results.xs[0]]
                joint_state_msg = JointState()
                joint_state_msg.name = self.last_sensor_msg.joint_state.name
                joint_state_msg.position = x_desired[:self.mpc.n_q].copy()
                joint_state_msg.velocity = x_desired[self.mpc.n_q:].copy()
                sensor_msg.joint_state = joint_state_msg
                

                control_msg.initial_state =  sensor_msg # self.last_sensor_msg #

                sensor_x1_msg = Sensor()
                self.x_desired = [float(val) for val in self.mpc.results.xs[1]]
                joint_state_x1_msg = JointState()
                joint_state_x1_msg.name = self.last_sensor_msg.joint_state.name
                joint_state_x1_msg.position = self.x_desired[:self.mpc.n_q].copy()
                joint_state_x1_msg.velocity = self.x_desired[self.mpc.n_q:].copy()
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
        # Joint feedback
        position = list(msg.joint_state.position)
        position = np.array(position)

        velocity = list(msg.joint_state.velocity)
        velocity = np.array(velocity)

        # PErfect feedback
        # position = np.array(self.x_desired[:7])
        # velocity = np.array(self.x_desired[self.mpc.n_q:])

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
        marker.header.frame_id = "fr3_link0";
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

    def waypoints_to_marker(self, waypoints):

        points_list = []
        marker = Marker()
        marker.header.frame_id = "fr3_link0";
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

