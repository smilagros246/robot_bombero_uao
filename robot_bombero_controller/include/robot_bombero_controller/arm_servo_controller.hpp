#ifndef ARM_SERVO_CONTROLLER__ARM_SERVO_CONTROLLER_HPP_
#define ARM_SERVO_CONTROLLER__ARM_SERVO_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"


namespace arm_servo_controller
{

class ArmServoController : public controller_interface::ControllerInterface
{
public:
  ArmServoController() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;


  void set_joint_position_targets(const std::vector<double>& targets);
  void set_water_pump_effort(double effort);

private:
  std::vector<std::string> joint_names_;
  std::vector<hardware_interface::LoanedCommandInterface> joint_handles_;
  std::unique_ptr<hardware_interface::LoanedCommandInterface> water_pump_handle_;

  std::vector<double> joint_position_targets_;
  double water_pump_effort_{0.0};

  std::string water_pump_joint_;
};

}  // namespace arm_servo_controller

#endif  // ARM_SERVO_CONTROLLER__ARM_SERVO_CONTROLLER_HPP_
