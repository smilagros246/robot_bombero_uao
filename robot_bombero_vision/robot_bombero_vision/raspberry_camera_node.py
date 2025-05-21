import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2
import time
import smbus  # para leer desde el AMG8833

# Importación correcta del sensor desde tu paquete
from robot_bombero_vision.amg8833_i2c import AMG8833

class LightweightCamThermalNode(Node):
    def __init__(self):
        super().__init__('cam_thermal_node')
        self.bridge = CvBridge()

        # Publicadores
        self.rgb_pub = self.create_publisher(Image, 'camera/rgb/image_raw', 10)
        self.thermal_pub = self.create_publisher(Image, 'camera/thermal/image_raw', 10)

        # Cámara USB
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)
        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir la cámara USB')
            exit(1)

        # Inicializar sensor AMG8833
        self.amg = None
        for addr in [0x69, 0x68]:
            try:
                self.amg = AMG8833(addr=addr)
                break
            except Exception as e:
                self.get_logger().warn(f'Error al intentar con dirección {hex(addr)}: {e}')
                continue

        if self.amg is None:
            self.get_logger().error('Sensor AMG8833 no detectado')

        # Temporizador (5 Hz)
        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        # Imagen RGB
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header = Header(stamp=now)
            self.rgb_pub.publish(msg)

        # Imagen térmica
        if self.amg:
            status, pixels = self.amg.read_temp(64)
            if not status:
                data = np.array(pixels).reshape((8, 8))
                norm = np.clip((data - 20) / 60.0 * 255, 0, 255)
                thermal_img = norm.astype(np.uint8)
                thermal_img = cv2.resize(thermal_img, (64, 64), interpolation=cv2.INTER_NEAREST)

                msg = self.bridge.cv2_to_imgmsg(thermal_img, encoding='mono8')  # más ligero
                msg.header = Header(stamp=now)
                self.thermal_pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LightweightCamThermalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
