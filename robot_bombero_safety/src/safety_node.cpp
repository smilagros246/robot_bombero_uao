#include "robot_bombero_safety/safety_node.hpp"

using std::placeholders::_1;

namespace robot_bombero_safety
{

SafetyNode::SafetyNode() : Node("safety_node"), ir_threshold_(950.0), tilt_threshold_(45.0)
{
  // Publicadores
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  safety_status_pub_ = this->create_publisher<std_msgs::msg::String>("/safety_status", 10);

  // Suscripciones
  cmd_vel_raw_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_raw", 10, std::bind(&SafetyNode::cmd_vel_raw_callback, this, _1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data_raw", 10, std::bind(&SafetyNode::imu_callback, this, _1));

  ir_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/ir/data", 10, std::bind(&SafetyNode::ir_callback, this, _1));

  // Timer para revisar la seguridad cada 100ms
  safety_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&SafetyNode::safety_check, this));
}

void SafetyNode::cmd_vel_raw_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_raw_ = *msg;
}

void SafetyNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_data_ = *msg;
}

void SafetyNode::ir_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  ir_data_ = *msg;
}

bool SafetyNode::detect_obstacle()
{
  if (ir_data_.data.empty())
  {
    RCLCPP_WARN(this->get_logger(), "No IR data received yet.");
    return false;
  }

  for (size_t i = 0; i < ir_data_.data.size(); ++i)
  {
    if (ir_data_.data[i] > ir_threshold_)
    {
      RCLCPP_WARN(this->get_logger(), "Obstacle detected! IR sensor %zu value: %f", i, ir_data_.data[i]);
      return true;
    }
  }
  return false;
}

bool SafetyNode::detect_tilt()
{
  // Umbral para velocidad angular (rad/s), ajusta según tu robot
  const double gyro_threshold = 10.0;  // por ejemplo 1 rad/s (~57°/s)

  double wx = imu_data_.angular_velocity.x;
  double wy = imu_data_.angular_velocity.y;
  double wz = imu_data_.angular_velocity.z;

  // Detectar giro excesivo en eje x o y (balanceo o cabeceo)
  if (std::abs(wx) > gyro_threshold || std::abs(wy) > gyro_threshold)
  {
    RCLCPP_WARN(this->get_logger(), "Inclinacion detectada por giro! wx=%.2f, wy=%.2f", wx, wy);
    return true;
  }
  return false;
}


void SafetyNode::safety_check()
{
  bool obstacle = detect_obstacle();
  bool tilt = detect_tilt();

  geometry_msgs::msg::Twist cmd_to_pub;

  std_msgs::msg::String status_msg;

  if (obstacle || tilt)
  {
    // Detener el robot
    cmd_to_pub.linear.x = 0.0;
    cmd_to_pub.linear.y = 0.0;
    cmd_to_pub.linear.z = 0.0;
    cmd_to_pub.angular.x = 0.0;
    cmd_to_pub.angular.y = 0.0;
    cmd_to_pub.angular.z = 0.0;

    status_msg.data = "Safety alert: Obstacle or tilt detected!";
  }
  else
  {
    cmd_to_pub = cmd_vel_raw_;
    status_msg.data = "System Safe";
  }

  cmd_vel_pub_->publish(cmd_to_pub);
  safety_status_pub_->publish(status_msg);
}

}  // namespace robot_bombero_safety

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_bombero_safety::SafetyNode>());
  rclcpp::shutdown();
  return 0;
}
