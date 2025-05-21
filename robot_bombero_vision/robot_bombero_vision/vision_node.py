import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import scipy.ndimage

class ThermalProcessorNode(Node):
    def __init__(self):
        super().__init__('thermal_processor_node')

        # Parámetros para los topics
        self.declare_parameter('thermal_topic', '/camera/thermal/image_raw')
        self.declare_parameter('rgb_topic', '/camera/rgb/image_raw')
        thermal_topic = self.get_parameter('thermal_topic').get_parameter_value().string_value
        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value

        # Subscripciones
        self.subscription_thermal = self.create_subscription(
            Image,
            thermal_topic,
            self.thermal_callback,
            10
        )
        self.subscription_rgb = self.create_subscription(
            Image,
            rgb_topic,
            self.rgb_callback,
            10
        )

        self.get_logger().info(f"Suscrito a cámara térmica: {thermal_topic}")
        self.get_logger().info(f"Suscrito a cámara RGB: {rgb_topic}")

        # Utilidades
        self.bridge = CvBridge()
        self.interp_factor = 5
        self.latest_rgb_frame = None

        # Ventana de visualización
        cv2.namedWindow("Vista combinada", cv2.WINDOW_NORMAL)

    def rgb_callback(self, msg):
        try:
            self.latest_rgb_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error procesando imagen RGB: {e}")

    def thermal_callback(self, msg):
        try:
            thermal_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(thermal_image, cv2.COLOR_BGR2GRAY)
            thermal_resized = cv2.resize(gray, (8, 8), interpolation=cv2.INTER_LINEAR)
            data_interp = scipy.ndimage.zoom(thermal_resized, self.interp_factor, order=3)

            # Punto más caliente
            max_idx = np.unravel_index(np.argmax(data_interp), data_interp.shape)
            max_temp = data_interp[max_idx]

            # Colormap y anotación
            thermal_colored = cv2.applyColorMap(data_interp.astype(np.uint8), cv2.COLORMAP_INFERNO)
            cv2.circle(thermal_colored, (max_idx[1], max_idx[0]), 5, (255, 255, 255), 2)
            cv2.putText(thermal_colored, f'{max_temp:.1f}°C', (max_idx[1]+5, max_idx[0]-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

            # === ESPACIO PARA INTEGRAR MODELO DE DETECCIÓN DE FUEGO ===
            # if self.latest_rgb_frame is not None:
            #     fire_detected = fire_model.predict(thermal_colored, self.latest_rgb_frame)
            #     if fire_detected:
            #         self.get_logger().info("¡Fuego detectado!")

            # Visualización combinada (solo si hay RGB disponible)
            if self.latest_rgb_frame is not None:
                rgb_resized = cv2.resize(self.latest_rgb_frame, (thermal_colored.shape[1], thermal_colored.shape[0]))
                combined = np.hstack((rgb_resized, thermal_colored))
                cv2.imshow("Vista combinada", combined)
            else:
                cv2.imshow("Vista combinada", thermal_colored)

            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error procesando imagen térmica: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ThermalProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
