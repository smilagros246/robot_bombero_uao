#include "robot_bombero_hardware/firebot_hardware_interface.hpp"
#include <libserial/SerialStream.h>
#include <iostream>
#include <cstring>
#include <unistd.h>

namespace firebot_hardware
{

hardware_interface::CallbackReturn FireBotHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Configura el puerto serial
  try
  {
    
    port_name_ = info_.hardware_parameters.at("port_name");
    serial_port_.Open(port_name_);
    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  }
  catch (const LibSerial::OpenFailed &)
  {
    std::cerr << "No se pudo abrir el puerto serial correctamente." << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Inicializa vectores de comandos y estados
  velocity_states_.resize(4, 0.0);
  velocity_commands_.resize(4, 0.0);
  position_states_.resize(4, 0.0);
  position_commands_.resize(4, 0.0);
  ir_values_.resize(3, 0.0);
  imu_values_.resize(3, 0.0);
  water_pump_command_ = 0.0;


  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FireBotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto& joint = info_.joints[i];

    if (i < 4) // Primeros 4 son ruedas (estado dummy)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint.name, "velocity", &velocity_states_[i]));
    }
    else if (i < 8) // Los otros 4 son servos (estado dummy)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint.name, "position", &position_states_[i - 4]));
    }
  }

  

  // Interfaces de estado para los sensores IR
  state_interfaces.emplace_back(hardware_interface::StateInterface("ir_left", "distance", &ir_values_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("ir_center", "distance", &ir_values_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("ir_right", "distance", &ir_values_[2]));

  // Interfaces de estado para IMU (puedes ajustar los nombres según cómo se usen)
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_x", "acceleration", &imu_values_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_y", "acceleration", &imu_values_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_z", "acceleration", &imu_values_[2]));


  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FireBotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto& joint = info_.joints[i];

    if (i < 4) // Primeros 4 joints = ruedas
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joint.name, "velocity", &velocity_commands_[i]));
    }
    else if (i < 8) // Siguientes 4 joints = servos
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joint.name, "position", &position_commands_[i - 4]));
    }
  }

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("joint_water_pump", "effort", &water_pump_command_));

  return command_interfaces;
}

hardware_interface::return_type FireBotHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  char buffer[256];
  serial_port_.read(buffer, sizeof(buffer));
  parseSensorData(buffer);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FireBotHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::ostringstream cmd;
  cmd << "CMD:WHEELS:";
  for (size_t i = 0; i < 4; ++i) {
    cmd << static_cast<int>(velocity_commands_[i]);
    if (i < 3) cmd << ",";
  }
  cmd << ";";

  cmd << "SERVOS:";
  for (size_t i = 0; i < 4; ++i) {
    cmd << static_cast<int>(position_commands_[i]);
    if (i < 3) cmd << ",";
  }
  cmd << ";";

  // Control de la bomba de agua con la variable water_pump_command_
  if (water_pump_command_ < 0.3) cmd << "PUMP:PWM_BAJO;";
  else if (water_pump_command_ < 0.7) cmd << "PUMP:PWM_MEDIO;";
  else cmd << "PUMP:PWM_ALTO;";

  cmd << "\n";

  serial_port_.write(cmd.str().c_str(), cmd.str().size());
  return hardware_interface::return_type::OK;

}

void FireBotHardwareInterface::parseSensorData(const std::string &data)
{
  size_t ir_pos = data.find("IR:");
  size_t imu_pos = data.find("IMU:");

  if (ir_pos != std::string::npos)
  {
    sscanf(data.c_str() + ir_pos, "IR:%lf,%lf,%lf", &ir_values_[0], &ir_values_[1], &ir_values_[2]);
  }

  if (imu_pos != std::string::npos)
  {
    sscanf(data.c_str() + imu_pos, "IMU:%lf,%lf,%lf", &imu_values_[0], &imu_values_[1],&imu_values_[2]);
  }
}

} // namespace firebot_hardware
