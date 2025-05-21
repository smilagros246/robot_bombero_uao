#ifndef FIREBOT_HARDWARE_INTERFACE_HPP
#define FIREBOT_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/visibility_control.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_bombero_hardware/serial_port.hpp" 

#include "string"
#include "unordered_map"
#include "vector"

using hardware_interface::return_type;
namespace robot_bombero_hardware
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC FireBotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FireBotHardwareInterface)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
  //Acceder a valores de los sensores
  std::vector<double>& getIRValues() { return ir_states_; }
  std::vector<double>& getIMUValues() { return imu_states_; }

protected:
  void parseSensorData(const std::string &data);
  bool readAndParsePacket();

  std::string generar_comando_serial();
  std::unique_ptr<SerialPort> serial_;

  std::array<std::string, 4> wheel_joints_ = { "link_fl_wheel_joint", "link_fr_wheel_joint", "link_rl_wheel_joint", "link_rr_wheel_joint" };
  std::array<std::string, 4> servo_joints_ = { "joint_waist", "joint_shoulder", "joint_elbow", "joint_wrist" };
  std::string water_pump_joint_ = "joint_water_pump";
  std::array<std::string, 3> gpio_ir_names_ = { "sharp_front", "sharp_left", "sharp_right" };

  // Comandos y estados para las ruedas (velocity)
  std::vector<double> joint_velocity_commands_;
  std::vector<double> joint_velocity_states_;

  // Comandos y estados para los servos (position)
  std::vector<double> joint_position_commands_;
  std::vector<double> joint_position_states_;

  //Comando de la bomba
  double water_pump_command_;
  double water_pump_state_;
  double water_pump_position_state_;

  // Para simular estados actuales de posicion de las ruedas
  std::vector<double> joint_position_states_wheels_;
  std::vector<double> joint_position_commands_wheels_;



  // Sensores auxiliares
  std::vector<double> ir_states_;
  std::vector<double> imu_states_;
  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
    {"position", {}}, {"velocity", {}}};
};

} // namespace robot_bombero_hardware

#endif // FIREBOT_HARDWARE_INTERFACE_HPP
