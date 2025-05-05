import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('image_topic_rgb', '/camera/rgb/image_raw')
        self.declare_parameter('image_topic_thermal', '/camera/thermal/image_raw')
        self.image_topic_rgb = self.get_parameter('image_topic_rgb').get_parameter_value().string_value
        self.image_topic_thermal = self.get_parameter('image_topic_thermal').get_parameter_value().string_value

        self.subscriber_rgb = self.create_subscription(
            Image,
            self.image_topic_rgb,
            self.rgb_image_callback,
            10)
        self.subscriber_thermal = self.create_subscription(
            Image,
            self.image_topic_thermal,
            self.thermal_image_callback,
            10)

        self.bridge = CvBridge()

    def rgb_image_callback(self, msg):
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("RGB Image", rgb_image)
        cv2.waitKey(1)

    def thermal_image_callback(self, msg):
        thermal_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        cv2.imshow("Thermal Image", thermal_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
