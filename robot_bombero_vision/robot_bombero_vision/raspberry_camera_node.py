import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import time, sys
from . import amg8833_i2c  

class RaspberryCameraNode(Node):
    def __init__(self):
        super().__init__('raspberry_camera_node')

        self.declare_parameter('image_topic_rgb', '/camera/rgb/image_raw')
        self.declare_parameter('image_topic_thermal', '/camera/thermal/image_raw')

        image_topic_rgb = self.get_parameter('image_topic_rgb').get_parameter_value().string_value
        image_topic_thermal = self.get_parameter('image_topic_thermal').get_parameter_value().string_value

        self.rgb_pub = self.create_publisher(Image, image_topic_rgb, 10)
        self.thermal_pub = self.create_publisher(Image, image_topic_thermal, 10)
        self.bridge = CvBridge()

        # Inicializar cámara Pi
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara Pi")
            exit(1)

        # Inicializar sensor térmico AMG8833 (esperar 1 segundo como recomienda la guía)
        self.amg = None
        t0 = time.time()
        while (time.time() - t0) < 1:
            try:
                self.amg = amg8833_i2c.AMG8833(addr=0x69)
                break
            except:
                try:
                    self.amg = amg8833_i2c.AMG8833(addr=0x68)
                    break
                except:
                    pass

        if self.amg is None:
            self.get_logger().error("No se encontró el sensor AMG8833. Verifica el cableado.")
        else:
            time.sleep(0.1)  # Dejar estabilizar el sensor

        self.timer = self.create_timer(0.2, self.publish_data)

    def publish_data(self):
        # Imagen RGB
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header = Header(stamp=self.get_clock().now().to_msg())
            self.rgb_pub.publish(msg)

        # Imagen térmica
        if self.amg:
            status, pixels = self.amg.read_temp(64)
            if not status:
                thermal = np.array(pixels).reshape((8, 8))

                thermal = np.clip(thermal, 20, 80)
                norm = (thermal - 20) / 60.0 * 255
                thermal_img = norm.astype(np.uint8)

                thermal_img = cv2.resize(thermal_img, (240, 240), interpolation=cv2.INTER_CUBIC)
                thermal_colored = cv2.applyColorMap(thermal_img, cv2.COLORMAP_JET)

                msg = self.bridge.cv2_to_imgmsg(thermal_colored, encoding='bgr8')
                msg.header = Header(stamp=self.get_clock().now().to_msg())
                self.thermal_pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RaspberryCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
