import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def detect_fire_by_heat(gray_thermal, threshold=200, min_pixels=5):
    """
    Detecta posible fuego si hay suficientes píxeles térmicos calientes.
    :param thermal_image: Imagen térmica en escala de grises (mono8)
    :param threshold: Umbral mínimo de temperatura (0-255)
    :param min_pixels: Número mínimo de píxeles calientes para confirmar fuego
    :return: True si hay fuego, False si no
    """
    hot_pixels = np.sum(gray_thermal > threshold)
    return hot_pixels >= min_pixels

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
            10
        )

        self.subscriber_thermal = self.create_subscription(
            Image,
            self.image_topic_thermal,
            self.thermal_image_callback,
            10
        )

        self.bridge = CvBridge()
        self.rgb_image = None
        self.thermal_image = None

    def rgb_image_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("RGB Image", self.rgb_image)
        cv2.waitKey(1)

        # Aquí se insertará la lógica de IA para detectar fuego en imagen RGB
        # if detect_fire(self.rgb_image):
        #     self.get_logger().info("Posible fuego detectado en RGB")

    def thermal_image_callback(self, msg):
        self.thermal_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray_thermal = cv2.cvtColor(self.thermal_image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Thermal Image", self.thermal_image)
        cv2.waitKey(1)

        if detect_fire_by_heat(self.thermal_image):
            self.get_logger().warn("Calor extremo detectado — Posible fuego (sensor térmico)")

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
