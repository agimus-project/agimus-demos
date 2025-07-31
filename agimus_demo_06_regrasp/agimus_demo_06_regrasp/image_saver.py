import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class ImageSaver(Node):
    def __init__(self):
        super().__init__("image_saver")
        self.subscription = self.create_subscription(
            Image,
            "/camera2/color/image_raw",  # Change this to your topic
            self.listener_callback,
            10,
        )
        self.bridge = CvBridge()
        self.img_count = 0
        os.makedirs("output_images", exist_ok=True)
        for filename in os.listdir("output_images"):
            file_path = os.path.join("output_images", filename)
            if os.path.isfile(file_path):
                os.remove(file_path)

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        filename = f"output_images/frame_{self.img_count:05d}.png"
        cv2.imwrite(filename, cv_image)
        self.img_count += 1


def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    image_saver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
