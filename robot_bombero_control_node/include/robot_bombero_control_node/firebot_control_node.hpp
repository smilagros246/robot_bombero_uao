#ifndef FIREBOT_CONTROL_NODE_HPP_
#define FIREBOT_CONTROL_NODE_HPP_

#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "robot_bombero_hardware/firebot_hardware_interface.hpp"

namespace robot_bombero_control_node
{

class FirebotControlNode : public rclcpp::Node
{
public:
  explicit FirebotControlNode(std::shared_ptr<robot_bombero_hardware::FireBotHardwareInterface> hw_interface);

private:
  void publish_sensors();

  std::shared_ptr<robot_bombero_hardware::FireBotHardwareInterface> hw_interface_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ir_front_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ir_left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ir_right_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_bombero_control_node

#endif  // FIREBOT_CONTROL_NODE_HPP_
