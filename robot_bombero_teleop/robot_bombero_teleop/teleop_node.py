#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        self.chassis_mode = True
        self.joint_positions = [1.57,1.57,1.57,1.57]

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.joint_pub = self.create_publisher(Float64MultiArray, '/joint_positions', 10)
        self.pump_pub = self.create_publisher(Float64, '/water_pump_cmd', 10)

        self.last_back = 0
        self.last_twist = Twist()
        self.last_pump_value = 0.0  # Inicializado en cero para la bomba

        self.get_logger().info("TeleopNode inicializado. Modo: CHASIS")
        self.get_logger().info("""
╭────────────────────────── Xbox Controller Layout ─────────────────────────╮
│ Axes:                                                                     │
│   [0] Left Stick Horizontal  →  linear.y (mov lateral)                    │
│   [1] Left Stick Vertical    →  linear.x (mov adelante/atrás)            │
│   [3] Right Stick Horizontal →  angular.z (rotación)                     │
│   [5] RT (Trigger Derecho)   →  Control bomba (presión)                  │
│   [6] D-pad Horizontal       →  Joint 1 (base)                           │
│   [7] D-pad Vertical         →  Joint 2 (hombro)                         │
│                                                                         │
│ Buttons:                                                                 │
│   [0] A → Disminuir Joint 3 (codo)                                       │
│   [1] B → Aumentar Joint 3 (codo)                                        │
│   [2] X → Disminuir Joint 4 (aspersor)                                   │
│   [3] Y → Aumentar Joint 4 (aspersor)                                    │
│   [4] LB → Bomba nivel alto (con RT)                                     │
│   [5] RB → Bomba nivel bajo                                              │
│   [6] Back → Cambia entre modo CHASIS / BRAZO                            │
╰──────────────────────────────────────────────────────────────────────────╯
        """)

    def joy_callback(self, msg: Joy):
        if msg.buttons[6] == 1 and self.last_back == 0:
            self.chassis_mode = not self.chassis_mode
            modo = "CHASIS" if self.chassis_mode else "BRAZO"
            self.get_logger().info(f"Cambio de modo: {modo}")
        self.last_back = msg.buttons[6]

        # Manejo de joints: actualizar sólo si modo brazo
        if not self.chassis_mode:
            self.update_joints(msg)
            self.last_twist = Twist()
            self.get_logger().info("Twist puesto en cero al cambiar a modo BRAZO")
            self.publish_twist()  # Publicar el twist en cero inmediatamente
        # Siempre publicar joints aunque no cambien (para que no deje de publicar)
        self.publish_joints()

        # Manejo bomba: actualizar sólo si modo brazo, en modo chasis bomba a 0
        if not self.chassis_mode:
            self.update_pump(msg)
        else:
            # modo chasis siempre bomba 0
            if self.last_pump_value != 0.0:
                self.last_pump_value = 0.0

        # Siempre publicar bomba (incluso si no cambia)
        self.publish_pump()

        # Manejo velocidades: actualizar sólo si modo chasis
        if self.chassis_mode:
            self.update_twist(msg)
        # Siempre publicar velocidades (incluso si no cambia)
        self.publish_twist()
        
    
    def clamp_angle_rad(self, angle, min_deg, max_deg):
        min_rad = math.radians(min_deg)
        max_rad = math.radians(max_deg)
        return max(min(angle, max_rad), min_rad)

    def update_joints(self, msg: Joy):
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

    def publish_joints(self):
        joint_msg = Float64MultiArray()
        self.joint_positions[0] = self.clamp_angle_rad(self.joint_positions[0], 0, 180)
        self.joint_positions[1] = self.clamp_angle_rad(self.joint_positions[1], 0, 180)
        self.joint_positions[2] = self.clamp_angle_rad(self.joint_positions[2], 0, 180)
        self.joint_positions[3] = self.clamp_angle_rad(self.joint_positions[3], 0, 180)
        joint_msg.data = self.joint_positions
        self.joint_pub.publish(joint_msg)

    def update_pump(self, msg: Joy):
        rt_value = (1 - msg.axes[5]) / 2  # De [1, -1] a [0, 1]
        level = 0.0  # Default bomba apagada

        # Verifica botones bomba
        if msg.buttons[5]:
            level = 1.0  # RB → Bajo
        elif msg.buttons[4] and rt_value > 0.9:
            level = 3.0  # LB + RT → Alto
        elif rt_value > 0.1:
            level = 2.0  # Solo RT → Medio

        if level != self.last_pump_value:
            self.last_pump_value = level
            estado = "apagada" if level == 0.0 else f"Nivel {level}"
            self.get_logger().info(f"Bomba {estado}")

    def publish_pump(self):
        pump_msg = Float64()
        pump_msg.data = self.last_pump_value
        self.pump_pub.publish(pump_msg)

    def update_twist(self, msg: Joy):
        max_x = 0.3
        max_y = 0.3
        max_omega = 1.5
        twist = Twist()
        twist.linear.x = msg.axes[1] * max_x
        twist.linear.y = msg.axes[0] * max_y
        twist.angular.z = msg.axes[3] * max_omega

        # Solo actualiza si hay cambio (para no saturar)
        if (twist.linear.x != self.last_twist.linear.x or
            twist.linear.y != self.last_twist.linear.y or
            twist.angular.z != self.last_twist.angular.z):
            self.last_twist = twist
            self.get_logger().info(f"Velocidad: x={twist.linear.x:.2f}, y={twist.linear.y:.2f}, yaw={twist.angular.z:.2f}")

    def publish_twist(self):
        self.cmd_vel_pub.publish(self.last_twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
