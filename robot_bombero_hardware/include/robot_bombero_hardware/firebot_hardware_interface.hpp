#ifndef FIREBOT_HARDWARE_INTERFACE_HPP
#define FIREBOT_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/macros.hpp"
#include "hardware_interface/visibility_control.h"
#include <libserial/SerialStream.h>
#include <string>
#include <vector>

namespace firebot_hardware
{

class FireBotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FireBotHardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  //Acceder a valores de los sensores
  std::vector<double>& getIRValues() { return ir_values_; }
  std::vector<double>& getIMUValues() { return imu_values_; }

private:
  void parseSensorData(const std::string &data);

  // Comandos y estados para las ruedas (velocity)
  std::vector<double> velocity_commands_;
  std::vector<double> velocity_states_;

  // Comandos y estados para los servos (position)
  std::vector<double> position_commands_;
  std::vector<double> position_states_;

  //Comando de la bomba
  double water_pump_command_;


  // Sensores auxiliares
  std::vector<double> ir_values_;
  std::vector<double> imu_values_;

  // Puerto serial
  LibSerial::SerialStream serial_port_;
  std::string port_name_;
};

} // namespace firebot_hardware

#endif // FIREBOT_HARDWARE_INTERFACE_HPP
