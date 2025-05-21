#include "robot_bombero_controller/arm_servo_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "pluginlib/class_list_macros.hpp"

using config_type = controller_interface::interface_configuration_type;
namespace robot_bombero_controller
{

ArmServoController::ArmServoController()
: controller_interface::ControllerInterface()
{}

controller_interface::CallbackReturn ArmServoController::on_init()
{
  RCLCPP_INFO(get_node()->get_logger(), "on_init: Inicializando controlador de brazo...");

  joint_names_ = auto_declare<std::vector<std::string>>("joints", {
    "joint_waist",
    "joint_shoulder",
    "joint_elbow",
    "joint_wrist"
  });
  pump_joint_name_ = auto_declare<std::string>("pump_joint", "joint_water_pump");


  command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", {
    hardware_interface::HW_IF_POSITION
  });

  state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", {
    hardware_interface::HW_IF_POSITION
  });

  pump_command_interface_types_ = auto_declare<std::vector<std::string>>("pump_command_interface", {hardware_interface::HW_IF_EFFORT});
  pump_state_interface_types_ = auto_declare<std::vector<std::string>>("pump_state_interface", {hardware_interface::HW_IF_EFFORT});


  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ArmServoController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Interfaces para joints (posición)
  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : command_interface_types_) {
      config.names.push_back(joint_name + "/" + interface_type);
    }
  }

  for (const auto &interface_type : pump_command_interface_types_) {
  config.names.push_back(pump_joint_name_ + "/" + interface_type);
  }

  return config;
}

controller_interface::InterfaceConfiguration ArmServoController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_) {
    for (const auto & interface_type : state_interface_types_) {
      config.names.push_back(joint_name + "/" + interface_type);
    }
  }

  // Interfaz para bomba (esfuerzo)
  for (const auto &interface_type : pump_state_interface_types_) {
  config.names.push_back(pump_joint_name_ + "/" + interface_type);
  }

  return config;
}

controller_interface::CallbackReturn ArmServoController::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_configure: Configurando controlador de brazo...");

  joint_position_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/joint_positions", rclcpp::SystemDefaultsQoS(),
    std::bind(&ArmServoController::jointPositionCallback, this, std::placeholders::_1)
  );

  joint_position_buffer_.writeFromNonRT(std::vector<double>(joint_names_.size(), 0.0));

  pump_command_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
  "/water_pump_cmd", rclcpp::SystemDefaultsQoS(),
  std::bind(&ArmServoController::pumpCommandCallback, this, std::placeholders::_1));


  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmServoController::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_activate: Activando controlador de brazo...");

  joint_command_interfaces_.clear();
  pump_command_interface_ = nullptr;

  RCLCPP_INFO(get_node()->get_logger(), "Interfaces recibidas en on_activate():");
  for (const auto & interface : command_interfaces_) {
    RCLCPP_INFO(get_node()->get_logger(), " - joint: '%s', interface: '%s'", 
                interface.get_name().c_str(), interface.get_interface_name().c_str());
  }


  for (auto & interface : command_interfaces_) {
    std::string full_name = interface.get_name(); // ej. "joint_water_pump/effort"
    std::string joint_name = full_name.substr(0, full_name.find('/'));

    RCLCPP_INFO(get_node()->get_logger(), "Chequeando interfaz joint='%s', interface='%s'",
                full_name.c_str(), interface.get_interface_name().c_str());

    if (joint_name == pump_joint_name_) {
      RCLCPP_INFO(get_node()->get_logger(), " - El joint coincide con pump_joint_name_='%s'", pump_joint_name_.c_str());
    } else {
      RCLCPP_INFO(get_node()->get_logger(), " - El joint NO coincide con pump_joint_name_='%s'", pump_joint_name_.c_str());
    }

    if (std::find(pump_command_interface_types_.begin(), pump_command_interface_types_.end(),
                  interface.get_interface_name()) != pump_command_interface_types_.end()) {
      RCLCPP_INFO(get_node()->get_logger(), " - La interfaz '%s' está en pump_command_interface_types_", 
                  interface.get_interface_name().c_str());
    } else {
      RCLCPP_INFO(get_node()->get_logger(), " - La interfaz '%s' NO está en pump_command_interface_types_", 
                  interface.get_interface_name().c_str());
    }

    if (joint_name == pump_joint_name_ &&
        std::find(pump_command_interface_types_.begin(), pump_command_interface_types_.end(),
                  interface.get_interface_name()) != pump_command_interface_types_.end()) {
      pump_command_interface_ = &interface;
      RCLCPP_INFO(get_node()->get_logger(), " - pump_command_interface_ asignado correctamente");
    } else {
      joint_command_interfaces_.push_back(std::ref(interface));
    }
  }


  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmServoController::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_deactivate: Desactivando controlador de brazo...");

  joint_position_sub_.reset();
  // Liberar interfaces de comando para permitir que otros controladores las usen
  release_interfaces();

  return controller_interface::CallbackReturn::SUCCESS;
}

void ArmServoController::jointPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() == joint_names_.size()) {
    joint_position_buffer_.writeFromNonRT(msg->data);
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "Cantidad de posiciones (%ld) no coincide con el número de articulaciones (%ld).", 
                msg->data.size(), joint_names_.size());
  }
}

void ArmServoController::pumpCommandCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  pump_command_ = msg->data;
}



controller_interface::return_type ArmServoController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto joint_positions = joint_position_buffer_.readFromRT();

  if (joint_positions->size() != joint_command_interfaces_.size()) {
    RCLCPP_WARN(get_node()->get_logger(), "No se aplicaron comandos: cantidad de posiciones no coincide.");
    return controller_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joint_command_interfaces_.size(); ++i) {
    joint_command_interfaces_[i].get().set_value((*joint_positions)[i]);
  }

  if (pump_command_interface_) {
  pump_command_interface_->set_value(pump_command_);
  } 

  return controller_interface::return_type::OK;
}

}  // namespace robot_bombero_controller

PLUGINLIB_EXPORT_CLASS(robot_bombero_controller::ArmServoController, controller_interface::ControllerInterface)
