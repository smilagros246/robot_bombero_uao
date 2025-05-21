#pragma once

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp/subscription.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include <string>
#include <algorithm> 
#include <vector>

namespace robot_bombero_controller
{

class ArmServoController : public controller_interface::ControllerInterface
{
public:
  ArmServoController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

protected:
  void jointPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void pumpCommandCallback(const std_msgs::msg::Float64::SharedPtr msg);

  std::string pump_joint_name_;
  std::vector<std::string> joint_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_state_interface_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;
  std::vector<std::string> pump_command_interface_types_;
  std::vector<std::string> pump_state_interface_types_;
  


  // Cambiar este puntero a LoanedCommandInterface
  hardware_interface::LoanedCommandInterface* pump_command_interface_ = nullptr;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pump_command_sub_;
  std::atomic<double> pump_command_{0.0};

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_sub_;
  realtime_tools::RealtimeBuffer<std::vector<double>> joint_position_buffer_;
};

}  // namespace robot_bombero_controller
