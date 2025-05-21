#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace robot_bombero_controller
{

class SensorStatePublisherController : public controller_interface::ControllerInterface
{
public:
  SensorStatePublisherController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> imu_states_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> ir_states_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32MultiArray>::SharedPtr ir_pub_;
};

}  // namespace robot_bombero_controller
