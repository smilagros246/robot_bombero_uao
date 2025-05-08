#include "robot_bombero_controller/arm_servo_controller.hpp"

#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "pluginlib/class_list_macros.hpp"

namespace arm_servo_controller
{

controller_interface::CallbackReturn ArmServoController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmServoController::on_configure(const rclcpp_lifecycle::State &)
{
  // Leer nombres de articulaciones
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No se especificaron nombres de articulaciones.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Leer nombre del joint de la bomba
  get_node()->declare_parameter<std::string>("water_pump_joint", "joint_water_pump");
  get_node()->get_parameter("water_pump_joint", water_pump_joint_);

  // Inicializar objetivos
  joint_position_targets_.resize(joint_names_.size(), 0.0);
  water_pump_effort_ = 0.0;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ArmServoController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  std::vector<std::string> joint_position_interfaces;
  for (const auto &joint : joint_names_) {
    joint_position_interfaces.push_back(joint + "/position");
  }

  joint_position_interfaces.push_back(water_pump_joint_ + "/effort");

  config.names = joint_position_interfaces;
  return config;
}

controller_interface::InterfaceConfiguration ArmServoController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn ArmServoController::on_activate(const rclcpp_lifecycle::State &)
{
  joint_handles_.clear();
  for (const auto &joint_name : joint_names_) {
    auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                           [&](const auto &interface) {
                             return interface.get_name() == joint_name + "/position";
                           });

    if (it == command_interfaces_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "No se encontró interfaz para %s", joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    joint_handles_.push_back(std::move(*it));
  }

  // Interfaz de la bomba
  auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                         [&](const auto &interface) {
                           return interface.get_name() == water_pump_joint_ + "/effort";
                         });

  if (it == command_interfaces_.end()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No se encontró interfaz de esfuerzo para %s", water_pump_joint_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  water_pump_handle_ = std::make_unique<hardware_interface::LoanedCommandInterface>(std::move(*it));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmServoController::on_deactivate(const rclcpp_lifecycle::State &)
{
  joint_handles_.clear();
  water_pump_handle_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ArmServoController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (joint_handles_.size() != joint_position_targets_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cantidad de joints y objetivos no coincide.");
    return controller_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joint_handles_.size(); ++i) {
    joint_handles_[i].set_value(joint_position_targets_[i]);
  }

  if (water_pump_handle_) {
    water_pump_handle_->set_value(water_pump_effort_);
  }

  return controller_interface::return_type::OK;
}

void ArmServoController::set_joint_position_targets(const std::vector<double>& targets)
{
  joint_position_targets_ = targets;
}

void ArmServoController::set_water_pump_effort(double effort)
{
  water_pump_effort_ = effort;
}

}  // namespace arm_servo_controller

PLUGINLIB_EXPORT_CLASS(arm_servo_controller::ArmServoController, controller_interface::ControllerInterface)
