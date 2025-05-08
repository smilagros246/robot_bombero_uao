#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        self.chassis_mode = True
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.joint_pub = self.create_publisher(Float64MultiArray, '/joint_positions', 10)
        self.pump_pub = self.create_publisher(Float64, '/water_pump_cmd', 10)

        self.last_back = 0
        self.last_twist = Twist()
        self.last_pump_value = None

        self.get_logger().info("TeleopNode inicializado. Modo: CHASIS")

    def joy_callback(self, msg: Joy):
        # Mostrar botones y ejes para pruebas
        self.get_logger().debug(f"Botones: {msg.buttons}, Ejes: {msg.axes}")

        if msg.buttons[6] == 1 and self.last_back == 0:
            self.chassis_mode = not self.chassis_mode
            modo = "CHASIS" if self.chassis_mode else "BRAZO"
            self.get_logger().info(f"Cambio de modo: {modo}")
        self.last_back = msg.buttons[6]

        if self.chassis_mode:
            self.handle_chassis(msg)
        else:
            self.handle_arm(msg)

        self.handle_pump(msg)

    def handle_chassis(self, msg: Joy):
        twist = Twist()
        twist.linear.x = -msg.axes[1]
        twist.linear.y = msg.axes[0]
        twist.angular.z = msg.axes[3]

        if twist != self.last_twist:
            self.cmd_vel_pub.publish(twist)
            self.last_twist = twist
            self.get_logger().info(f"Velocidad: x={twist.linear.x:.2f}, y={twist.linear.y:.2f}, yaw={twist.angular.z:.2f}")

    def handle_arm(self, msg: Joy):
        updated = False

        if msg.axes[6] != 0:
            self.joint_positions[0] += msg.axes[6] * 0.05
            updated = True
            self.get_logger().info(f"Servo 1 (base): {self.joint_positions[0]:.2f}")

        if msg.axes[7] != 0:
            self.joint_positions[1] += msg.axes[7] * 0.05
            updated = True
            self.get_logger().info(f"Servo 2 (hombro): {self.joint_positions[1]:.2f}")

        if msg.buttons[0]:  # A
            self.joint_positions[2] -= 0.05
            updated = True
            self.get_logger().info(f"Servo 3 (codo): {self.joint_positions[2]:.2f}")
        if msg.buttons[1]:  # B
            self.joint_positions[2] += 0.05
            updated = True
            self.get_logger().info(f"Servo 3 (codo): {self.joint_positions[2]:.2f}")

        if msg.buttons[2]:  # X
            self.joint_positions[3] -= 0.05
            updated = True
            self.get_logger().info(f"Servo 4 (aspersor): {self.joint_positions[3]:.2f}")
        if msg.buttons[3]:  # Y
            self.joint_positions[3] += 0.05
            updated = True
            self.get_logger().info(f"Servo 4 (aspersor): {self.joint_positions[3]:.2f}")

        if updated:
            joint_msg = Float64MultiArray()
            joint_msg.data = self.joint_positions
            self.joint_pub.publish(joint_msg)

    def handle_pump(self, msg: Joy):
        rt_value = (1 - msg.axes[5]) / 2  # De [1, -1] a [0, 1]
        level = None

        if msg.buttons[5]:
            level = 0.3  # Bajo
        elif msg.buttons[4] and rt_value > 0.9:
            level = 1.0  # Alto
        elif rt_value > 0.1:
            level = 0.6  # Medio

        if level is not None and level != self.last_pump_value:
            self.pump_pub.publish(Float64(data=level))
            self.last_pump_value = level
            self.get_logger().info(f"Bomba activada: Nivel {level}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
