import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_system_default
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from contact_graspnet_msgs.srv import GetSceneGrasps
from cv_bridge import CvBridge
import pickle


class RecorderNode(Node):
    def __init__(self):
        super().__init__("data_recorder")
        self.bridge = CvBridge()

        # Initialize variables to store the data
        self.happypose_data = None
        self.joints_data = None
        self.pointcloud_data = None
        self.image_data = None
        self.service_response = None

        # Create subscriptions to topics
        self.happypose_subscription = self.create_subscription(
            Detection2DArray,
            "/happypose/detections",
            self.happypose_callback,
            qos_profile_system_default,
        )
        self.joints_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joints_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.image_subscription = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback,
            qos_profile_system_default,
        )
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            "/camera/depth/color/points",
            self.pointcloud_callback,
            qos_profile_system_default,
        )

        # Create a service client
        self.client = self.create_client(
            GetSceneGrasps, "contact_graspnet/get_scene_grasps"
        )

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

        # Call the service
        self.call_service()

    def happypose_callback(self, msg):
        if self.happypose_data is None:
            self.happypose_data = msg
            self.get_logger().info("Received happypose data: ")
            self.stop_recording()

    def joints_callback(self, msg):
        if self.joints_data is None:
            self.joints_data = msg
            self.get_logger().info(f"Received joints data: {self.joints_data}")
            self.stop_recording()

    def image_callback(self, msg):
        if self.image_data is None:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.image_data = cv_image
            self.get_logger().info("Received image data:")
            self.stop_recording()

    def pointcloud_callback(self, msg):
        if self.pointcloud_data is None:
            self.pointcloud_data = msg
            self.get_logger().info("Received pointcloud data:")
            self.stop_recording()

    def call_service(self):
        # Call the service (replace with the appropriate request parameters)
        self.client.wait_for_service()
        self.get_logger().info("Graspnet service is available, calling...")
        request = GetSceneGrasps.Request()
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            self.service_response = response
            self.get_logger().info("Received service response:")
            self.stop_recording()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def stop_recording(self):
        # Check if we have collected one data point from each source
        if (
            self.happypose_data is not None
            and self.joints_data is not None
            and self.pointcloud_data is not None
            and self.image_data is not None
            and self.service_response is not None
        ):
            self.get_logger().info("All data collected. Stopping recording.")
            self.save_to_file()  # Save data to file
            self.get_logger().info("Saved. Destroying node")
            self.destroy_node()

    def save_to_file(self):
        # Prepare the data to save as a dictionary or other structure
        data = {
            "happypose_data": self.happypose_data,
            "joints_data": self.joints_data,
            "pointcloud": self.pointcloud_data,
            "image": self.image_data,
            "scene_grasps": self.service_response,
        }

        # Specify file path (this example saves to 'data_output.pkl')
        file_path = "recorded_scene01.pkl"

        try:
            with open(file_path, "wb") as file:  # Open in binary write mode
                # Use pickle to serialize the data
                pickle.dump(data, file)
            self.get_logger().info(f"Data saved to {file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save data to file: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
