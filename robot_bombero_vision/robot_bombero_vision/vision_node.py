import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import scipy.ndimage
from ultralytics import YOLO  # YOLOv8 para detección de fuego

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
        self.latest_rgb_frame = None
        self.latest_thermal_colored = None
        self.interp_factor = 5

        # Cargar modelo YOLOv8 para detección de fuego
        try:
            self.model = YOLO('fire_detector_mediplus.pt')  # ⚠️ Ajustar ruta
            self.get_logger().info("Modelo YOLOv8 cargado correctamente.")
        except Exception as e:
            self.get_logger().error(f"Error cargando el modelo YOLOv8: {e}")
            self.model = None

        # Ventana de visualización
        cv2.namedWindow("Vista combinada", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Vista combinada", 1280, 720)  # ⬅️ Ajusta a la resolución deseada


    def rgb_callback(self, msg):
        try:
            rgb_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_rgb_frame = rgb_frame.copy()

            if self.model is not None:
                results = self.model(rgb_frame)[0]

                if len(results.boxes) == 0:
                    self.get_logger().info("No se detectó fuego.")
                else:
                    self.get_logger().info(f"{len(results.boxes)} detección(es) encontradas.")
                    for box in results.boxes:
                        conf = box.conf.item()
                        cls_id = int(box.cls.item())
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        label = results.names[cls_id] if hasattr(results, 'names') else str(cls_id)

                        self.get_logger().info(f"🔥 Detección: {label} ({conf:.2f}) en [{x1},{y1},{x2},{y2}]")

                        if conf > 0.25:  # Umbral bajo para pruebas
                            cv2.rectangle(rgb_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                            cv2.putText(rgb_frame, f'{label} {conf:.2f}', (x1, y1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # Mostrar siempre aunque no haya detecciones
            if self.latest_thermal_colored is not None:
                thermal_resized = cv2.resize(self.latest_thermal_colored, (rgb_frame.shape[1], rgb_frame.shape[0]))
                combined = np.hstack((rgb_frame, thermal_resized))
                cv2.imshow("Vista combinada", combined)
            else:
                cv2.imshow("Vista combinada", rgb_frame)

            cv2.waitKey(1)

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

            # Guarda la imagen térmica coloreada para combinación futura
            self.latest_thermal_colored = thermal_colored

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

