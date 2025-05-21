#include "robot_bombero_control_node/firebot_control_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_bombero_control_node
{

FirebotControlNode::FirebotControlNode(std::shared_ptr<robot_bombero_hardware::FireBotHardwareInterface> hw_interface)
: Node("firebot_control_node"), hw_interface_(hw_interface)
{
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
  ir_front_pub_ = this->create_publisher<sensor_msgs::msg::Range>("ir/front", 10);
  ir_left_pub_ = this->create_publisher<sensor_msgs::msg::Range>("ir/left", 10);
  ir_right_pub_ = this->create_publisher<sensor_msgs::msg::Range>("ir/right", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&FirebotControlNode::publish_sensors, this));
}

void FirebotControlNode::publish_sensors()
{
  const std::vector<double>& imu_states = hw_interface_->getIMUValues();
  const std::vector<double>& ir_states = hw_interface_->getIRValues();

  RCLCPP_INFO(this->get_logger(), "Tamaño IMU: %zu, Tamaño IR: %zu", imu_states.size(), ir_states.size());


  if(imu_states.size() < 6 || ir_states.size() < 3)
  {
    RCLCPP_WARN(this->get_logger(), "Datos de sensores incompletos");
    return;
  }

  // Publicar IMU
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = this->now();
  imu_msg.header.frame_id = "link_imu";

  imu_msg.angular_velocity.x = imu_states[0];
  imu_msg.angular_velocity.y = imu_states[1];
  imu_msg.angular_velocity.z = imu_states[2];

  imu_msg.linear_acceleration.x = imu_states[3];
  imu_msg.linear_acceleration.y = imu_states[4];
  imu_msg.linear_acceleration.z = imu_states[5];

  imu_pub_->publish(imu_msg);

  // Publicar IR front
  sensor_msgs::msg::Range ir_front_msg;
  ir_front_msg.header.stamp = this->now();
  ir_front_msg.header.frame_id = "link_sharp_front";
  ir_front_msg.range = static_cast<float>(ir_states[0]);
  ir_front_msg.min_range = 0.02f;
  ir_front_msg.max_range = 2.0f;
  ir_front_msg.field_of_view = 0.5f;
  ir_front_pub_->publish(ir_front_msg);

  // Publicar IR left
  sensor_msgs::msg::Range ir_left_msg;
  ir_left_msg.header.stamp = this->now();
  ir_left_msg.header.frame_id = "link_sharp_left";
  ir_left_msg.range = static_cast<float>(ir_states[1]);
  ir_left_msg.min_range = 0.02f;
  ir_left_msg.max_range = 2.0f;
  ir_left_msg.field_of_view = 0.5f;
  ir_left_pub_->publish(ir_left_msg);

  // Publicar IR right
  sensor_msgs::msg::Range ir_right_msg;
  ir_right_msg.header.stamp = this->now();
  ir_right_msg.header.frame_id = "link_sharp_right";
  ir_right_msg.range = static_cast<float>(ir_states[2]);
  ir_right_msg.min_range = 0.02f;
  ir_right_msg.max_range = 2.0f;
  ir_right_msg.field_of_view = 0.5f;
  ir_right_pub_->publish(ir_right_msg);
}


}  // namespace robot_bombero_control_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto hw_interface = std::make_shared<robot_bombero_hardware::FireBotHardwareInterface>();
  auto node = std::make_shared<robot_bombero_control_node::FirebotControlNode>(hw_interface);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
