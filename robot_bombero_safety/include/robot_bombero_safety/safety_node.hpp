#ifndef ROBOT_BOMBERO_SAFETY__SAFETY_NODE_HPP_
#define ROBOT_BOMBERO_SAFETY__SAFETY_NODE_HPP_

#include <memory>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace robot_bombero_safety
{

class SafetyNode : public rclcpp::Node
{
public:
  SafetyNode();

private:
  // Callbacks
  void cmd_vel_raw_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void ir_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  void safety_check();

  // Detección
  bool detect_obstacle();
  bool detect_tilt();

  // Publicadores
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_status_pub_;

  // Suscriptores
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_raw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ir_sub_;

  // Variables para almacenar datos de sensores
  geometry_msgs::msg::Twist cmd_vel_raw_;
  sensor_msgs::msg::Imu imu_data_;
  std_msgs::msg::Float32MultiArray ir_data_;

  // Timer para evaluar la seguridad periódicamente
  rclcpp::TimerBase::SharedPtr safety_timer_;

  // Umbral para IR (ajustable)
  double ir_threshold_;
  // Umbral para ángulo de tilt (grados)
  double tilt_threshold_;
};

}  // namespace robot_bombero_safety

#endif  // ROBOT_BOMBERO_SAFETY__SAFETY_NODE_HPP_
