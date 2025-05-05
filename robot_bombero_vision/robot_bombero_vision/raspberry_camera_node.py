import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RaspberryCameraNode(Node):
    def __init__(self):
        super().__init__('raspberry_camera_node')
        self.publisher_rgb = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.publisher_thermal = self.create_publisher(Image, '/camera/thermal/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        # Captura la imagen RGB desde la cámara (ajustar el código según la cámara que uses)
        rgb_img = cv2.imread('/path/to/rgb_image.jpg')  # Sustituir por el flujo real
        if rgb_img is not None:
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_img, encoding="bgr8")
            self.publisher_rgb.publish(rgb_msg)

        # Captura la imagen térmica
        thermal_img = cv2.imread('/path/to/thermal_image.jpg')  # Sustituir por el flujo real
        if thermal_img is not None:
            thermal_msg = self.bridge.cv2_to_imgmsg(thermal_img, encoding="mono8")
            self.publisher_thermal.publish(thermal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RaspberryCameraNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
