from aligator_mpc import MPC, Params, PatternGenerator, TestTrajs
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from linear_feedback_controller_msgs.msg import Control, Sensor
from sensor_msgs.msg import JointState
from std_msgs.msg import MultiArrayDimension, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, QoSProfile
from pathlib import Path
import numpy as np
import sys
import aligator

# !! ===debug===
import time


class AligatorMPC(Node):
    def __init__(self):
        super().__init__('aligator_mpc')

        # ROS2 publishers & subscribers ============================================
        self.control_publisher = self.create_publisher(Control, "control", 10)
        self.mpc_pred_publisher = self.create_publisher(Marker, "mpc_prediction_endeffector",10)
        self.mpc_waypoints_publisher = self.create_publisher(Marker, "mpc_waypoints", 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(Sensor, 'sensor', self.sensor_callback, qos)
        self.launch_srv = self.create_service(Trigger, 'aligator_mpc/launch_mpc', self.launch_mpc_callback)
        self.mpc_started = False

        self.get_logger().info('Aligator MPC starting...')
        self.declare_parameter('config',"clearly a wrong value!!")
        config_path = self.get_parameter('config').value

        self.get_logger().info(config_path)
        mpc_parameters = Params(Path(config_path))        

        # Waypoints ================================================================
        # patternGen = PatternGenerator([0.5,0.5,0], (0.6,0,0.3))
        # mpc_waypoints = patternGen.generate_pattern('zigzag_curve',stride=0.05)
        
        test_trajs = TestTrajs()
        start = [0.2, 0.1, 0.4]
        end = [0.2, 1, 0.4]
        # mpc_waypoints = test_trajs.line(start, end)
        mpc_waypoints = test_trajs.sine(start_point=start,length=1,period=0.05,amplitude=0.1, dist_between_points=0.01, sine_axis="Y", ampl_axis="X")

        # Solver setup =============================================================
        #! DEBUG: check correctness of mpc setup
        # self.get_logger().info(str(parameters))
        # self.get_logger().info(str(positions))

        self.robot_state = None
        self.mpc = MPC(mpc_waypoints, mpc_parameters)
        self.first_mpc_iteration = True

        # Timestamps ===============================================================
        self.last_timestamp = Time(seconds=0, nanoseconds=0)
        timer_period = 0.01 # mpc_parameters.dt
        
        self.ctrl_timer = self.create_timer(timer_period, self.publish_control_callback)

        self.get_logger().info('End of Setup')

    def publish_control_callback(self):
            

            if self.mpc_started:
                
                
                if self.first_mpc_iteration:
                    self.first_mpc_iteration = False
                    self.mpc.initStages() # once the start pose is set the stages must be computed during before the first solver iteration
                    
                start = time.time()
                time_mpc = self.mpc.iterate(self.robot_state)
                # self.get_logger().info(f'mpc dt: {(1/time_mpc)}')
                

                #!!! ============================================================= debug
                if False:
                    xs_0 = self.mpc.results.xs.tolist()[0]
                    state = np.array([i for i in xs_0[:self.mpc.n_q]])
                    # self.get_logger().info(f'xs[0]:  {(xs_0)}')
                    # self.get_logger().info(f'MPC state: {(state)}')
                    mpc_endpoint = self.mpc.get_endpoint(state)
                    robot_endpoint = self.mpc.get_endpoint(self.robot_state[:self.mpc.n_q])
                    delta_endpoints = np.subtract(mpc_endpoint,robot_endpoint)
                    # self.get_logger().info(f'Robot endpoint:  {(robot_endpoint)}')
                    # self.get_logger().info(f'MPC endpoint:    {(mpc_endpoint)}')
                    # self.get_logger().info(f'Delta endpoints: {(delta_endpoints)}\n')
                #! ================================================================

                
                command = [v for v in self.mpc.results.us.tolist()[0]]
                # self.get_logger().info(f'MPC command: {command}')

                raw_feedback_gains = self.mpc.results.controlFeedbacks().tolist()[0]
                feedback_gains = self.flatten_feedback_gains(raw_feedback_gains)
                # self.get_logger().info(f'feddb gains: {(feedback_gains)}')


                #! ================== debug  predicted ee traj ==============
                if True:
                    xs_array = self.mpc.results.xs.tolist()
                    marker_array_msg = self.xs_to_marker(xs_array)
                    self.mpc_pred_publisher.publish(marker_array_msg)

                if False:
                    x_desired = self.mpc.results.xs.tolist()[0]
                    q_desired = [float(v) for v in x_desired[:self.mpc.n_q-2]] # convert numpy float to regular python float
                    self.debug_vizer.updatePose( x_desired[:self.mpc.n_q])
                    self.get_logger().info(f'q robot : {self.robot_state[:self.mpc.n_q-2]}')
                    self.get_logger().info(f'q desired: {(q_desired)}\n')

                    # v_desired = [float(v) for v in x_desired[self.mpc.n_q:-2]]
                    
                    # xs_no_ee[0][self.mpc.n_q - 1] = 0
                    # xs_no_ee[0][self.mpc.n_q - 2] = 0

                    # self.get_logger().info(f'type qs {type([float(v) for v in (q_desired)][0])}')
                    # self.get_logger().info(f'size of res us: {len(self.mpc.results.us.tolist())}')

                    # # publish to control
                    # self.get_logger().info(f'command: {command}')
                    # self.get_logger().info(f'ctrl feedback: {(feedback_gain)}') # list of riccatti gains tied to 
                #! ==========================================================
                if False:
                    x_desired = self.mpc.results.xs.tolist()[0]
                    self.debug_vizer.updatePose(self.robot_state[:self.mpc.n_q]) #x_desired[:self.mpc.n_q])

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
                control_msg.feedforward.data = [i*1 for i in command[:-2]] 

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
                control_msg.feedback_gain.data = [i*1 for i in feedback_gains]

                sensor_msg = Sensor()
                x_desired = [float(val) for val in self.mpc.results.xs.tolist()[0]]
                joint_state_msg = JointState()
                joint_state_msg.name = self.last_sensor_msg.joint_state.name
                joint_state_msg.position = x_desired[:self.mpc.n_q-2]
                joint_state_msg.velocity = x_desired[self.mpc.n_q:-2]
                sensor_msg.joint_state = joint_state_msg

                control_msg.initial_state = sensor_msg # self.last_sensor_msg # 

                self.control_publisher.publish(control_msg)
                end = time.time()
                self.get_logger().info(f' duree control {(end - start)}')
                # sys.exit()

    def sensor_callback(self, msg):
        self.last_sensor_msg = msg

        # Get robot state =========================================================
        position = list(msg.joint_state.position)
        position = np.array(position + [0.0, 0.0])


        velocity = list(msg.joint_state.velocity)
        velocity = np.array(velocity + [0.0, 0.0])

        # ==========================
        # position = np.array([-np.pi/2, -1, 0, -2.5, 0.0, 2, 0.0, 0.0, 0.0])
        # velocity = np.array([0.0, 0, 0, 0, 0.0, 0, 0.0, 0.0, 0.0])
        # ==========================

        self.robot_state = np.concatenate((position, velocity))
        self.mpc.setStartPose(self.robot_state[:self.mpc.n_q]) # update pinocchio model
        
        #!=================================== debug
        if False:
            # self.get_logger().info(f'Sensor message:: {str(msg)}')
            # self.get_logger().info(f'position:: {str(position)}') 
            # robot_endpoint = self.mpc.get_endpoint(position)
            # self.get_logger().info(f'Robot state endpoint: {(robot_endpoint)}')

            self.get_logger().info(f'Robot state: {(self.robot_state)}')

        if False:
            self.debug_vizer.updatePose(self.robot_state[:self.mpc.n_q])
        
    def launch_mpc_callback(self, request, response):
        self.mpc_started = True
        response.success = True # TODO : add checks to see if ready to launch
        self.ctrl_timer.reset()
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
        marker.scale.x = 0.01;


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
        for gains in raw_feedback_gains[:-2]:
            # self.get_logger().info(f'feddb gains: {(gains)}')
            gains = [float(k)for k in gains] # convert values to float 
            # self.get_logger().info(f'feddb gains: {type(gains)}')
            gains = gains[:self.mpc.n_q-2] + gains[self.mpc.n_q:-2] # remove gripper gains on q and on vs

            feedback_gains = feedback_gains + gains # add gains list at the end of the master list

        return feedback_gains

    def waypoints_to_marker(self, waypoints):
        pass
    #TODO

def main(args=None):
    rclpy.init(args=args)

    node = AligatorMPC()
    rclpy.spin(node)

if __name__ == "__main__":
    main()

