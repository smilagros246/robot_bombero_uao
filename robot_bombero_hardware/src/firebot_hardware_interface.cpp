#include "robot_bombero_hardware/firebot_hardware_interface.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>

namespace robot_bombero_hardware
{

CallbackReturn FireBotHardwareInterface ::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Configura el puerto serial
  serial_ = std::make_unique<SerialPort>("/dev/ttyUSB0", B115200);

  // Inicializa vectores de comandos y estados
  joint_velocity_states_.resize(wheel_joints_.size(), 0.0);
  joint_velocity_commands_.resize(wheel_joints_.size(), 0.0);
  joint_position_states_wheels_.resize(wheel_joints_.size(), 0.0);
  joint_position_states_.resize(servo_joints_.size(), 0.0);
  joint_position_commands_.resize(servo_joints_.size(), 0.0);
  ir_states_.resize(3,0);
  imu_states_.resize(6,0);
  water_pump_command_ = 0.0;
  water_pump_state_ = 0.0;
  water_pump_position_state_ = 0.0;


  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FireBotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (const auto& joint : info_.joints)
  {
    if (std::find(wheel_joints_.begin(), wheel_joints_.end(), joint.name) != wheel_joints_.end())
    {
      size_t idx = std::distance(wheel_joints_.begin(),
                                 std::find(wheel_joints_.begin(), wheel_joints_.end(), joint.name));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint.name, "velocity", &joint_velocity_states_[idx]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, "position", &joint_position_states_wheels_[idx]));
    }
    else if (std::find(servo_joints_.begin(), servo_joints_.end(), joint.name) != servo_joints_.end())
    {
      size_t idx = std::distance(servo_joints_.begin(),
                                 std::find(servo_joints_.begin(), servo_joints_.end(), joint.name));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint.name, "position", &joint_position_states_[idx]));
    }
    else if (joint.name == water_pump_joint_)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint.name, "effort", &water_pump_state_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint.name, "position", &water_pump_position_state_));
    }
  }

  // IMU
  state_interfaces.emplace_back("imu_sensor", "angular_velocity.x", &imu_states_[0]);
  state_interfaces.emplace_back("imu_sensor", "angular_velocity.y", &imu_states_[1]);
  state_interfaces.emplace_back("imu_sensor", "angular_velocity.z", &imu_states_[2]);
  state_interfaces.emplace_back("imu_sensor", "linear_acceleration.x", &imu_states_[3]);
  state_interfaces.emplace_back("imu_sensor", "linear_acceleration.y", &imu_states_[4]);
  state_interfaces.emplace_back("imu_sensor", "linear_acceleration.z", &imu_states_[5]);

  // Sensores IR
  for (size_t i = 0; i < gpio_ir_names_.size(); ++i)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
            gpio_ir_names_[i], "analog_input", &ir_states_[i]));
    RCLCPP_INFO(
        rclcpp::get_logger("FireBotHardwareInterface"),
        "Added GPIO state interface: %s", gpio_ir_names_[i].c_str());
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> FireBotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (const auto& joint : info_.joints)
  {
    if (std::find(wheel_joints_.begin(), wheel_joints_.end(), joint.name) != wheel_joints_.end())
    {
      size_t idx = std::distance(wheel_joints_.begin(),
                                 std::find(wheel_joints_.begin(), wheel_joints_.end(), joint.name));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joint.name, "velocity", &joint_velocity_commands_[idx]));
    }
    else if (std::find(servo_joints_.begin(), servo_joints_.end(), joint.name) != servo_joints_.end())
    {
      size_t idx = std::distance(servo_joints_.begin(),
                                 std::find(servo_joints_.begin(), servo_joints_.end(), joint.name));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joint.name, "position", &joint_position_commands_[idx]));
    }
    else if (joint.name == water_pump_joint_)
    {
      command_interfaces.emplace_back(
          hardware_interface::CommandInterface(joint.name, "effort", &water_pump_command_));
    }
  }

  return command_interfaces;
}


return_type FireBotHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Actualizar la posición estimada de las ruedas integrando la velocidad
  (void)time;  
  double dt = period.seconds();  // tiempo en segundos

  for (size_t i = 0; i < joint_velocity_states_.size(); ++i)
  {
    // Actualiza posición: theta_new = theta_old + omega * dt
    joint_position_states_wheels_[i] += joint_velocity_states_[i] * dt;
  }

  // Aquí puedes actualizar los estados de velocidad y posición de servos normalmente

  for (size_t i = 0; i < joint_velocity_states_.size(); ++i) {
    joint_velocity_states_[i] = joint_velocity_commands_[i];  // puedes actualizar velocidad con comando
  }

  for (size_t i = 0; i < joint_position_states_.size(); ++i) {
    joint_position_states_[i] = joint_position_commands_[i];
  }

  // Actualizar estado bomba
  water_pump_state_ = water_pump_command_;
  water_pump_position_state_ = 0;

  // Leer datos serial y parsear si hay sensores externos
  try {
  bool ok = readAndParsePacket();
  if (!ok) {
    RCLCPP_WARN(rclcpp::get_logger("FireBotHardwareInterface"), "No se pudo leer paquete válido");
  }
  } catch (const std::exception &e) {
    RCLCPP_WARN(rclcpp::get_logger("FireBotHardwareInterface"), "Error lectura serial: %s", e.what());
  }

return return_type::OK;
}



return_type FireBotHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  try {
    // Actualiza la "realimentación" simulada con los comandos actuales:

    // Para las ruedas (control por velocidad)
    for (size_t i = 0; i < joint_velocity_commands_.size(); ++i) {
      joint_velocity_states_[i] = joint_velocity_commands_[i];  // estado velocidad igual al comando enviado
    }

    // Para los servos (control por posición)
    for (size_t i = 0; i < joint_position_commands_.size(); ++i) {
      joint_position_states_[i] = joint_position_commands_[i];  // estado posición igual al comando enviado
    }

    // Para la bomba, asumo que solo interesa el comando (esfuerzo), el estado lo mantenemos igual
    water_pump_state_ = water_pump_command_;

    // Genera el comando serial y envíalo
  
    std::string mensaje = generar_comando_serial();
    // RCLCPP_INFO(rclcpp::get_logger("FireBotHardwareInterface"), "Mensaje binario de %zu bytes enviado por serial", mensaje.size());
    
    serial_->write_bytes(mensaje);  // Cambio importante aquí

  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("FireBotHardwareInterface"), "Error en write serial: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

return return_type::OK;
}



std::string FireBotHardwareInterface::generar_comando_serial() {
  std::string buffer;
  buffer.reserve(1 + 4*4 + 4*4 + 1 + 1 + 1);

  // Por ejemplo, 0x01 = usar servos, 0x00 = usar ruedas (puedes ajustar esto)
  uint8_t modo = 'S';
  buffer.push_back(modo);

  // Siempre enviar posiciones de servo (brazo)
  for (int i = 0; i < 4; ++i) {
    union { float f; uint8_t b[4]; } conv;
    conv.f = joint_position_commands_[i];
    buffer.insert(buffer.end(), conv.b, conv.b + 4);
  }

  // Siempre enviar velocidades ruedas
  for (int i = 0; i < 4; ++i) {
    union { float f; uint8_t b[4]; } conv;
    conv.f = joint_velocity_commands_[i];
    buffer.insert(buffer.end(), conv.b, conv.b + 4);
  }

  // Bomba
  buffer.push_back(static_cast<uint8_t>(water_pump_command_));

  // Checksum: suma de bytes desde índice 1 hasta antes del checksum
  uint8_t checksum = 0;
  for (size_t i = 1; i < buffer.size(); ++i) checksum += static_cast<uint8_t>(buffer[i]);
  buffer.push_back(checksum % 256);

  // Fin de mensaje
  buffer.push_back('#');

  return buffer;
}


void FireBotHardwareInterface::parseSensorData(const std::string &data) {
  // Validar tamaño y formato básico
  if (data.size() != 21 || data[0] != 'A' || (data[20] != '\n' && data[20] != '#')) {
    RCLCPP_WARN(rclcpp::get_logger("FireBotHardwareInterface"), "Mensaje inválido");
    return;
  }

  const uint8_t* raw = reinterpret_cast<const uint8_t*>(data.data());

  // Verificar checksum: suma de bytes [1..18] debe coincidir con byte 19
  uint8_t checksum = 0;
  for (int i = 1; i < 19; ++i) {
    checksum += raw[i];
  }
  // RCLCPP_INFO(rclcpp::get_logger("FireBotHardwareInterface"), "Checksum local: %d, recibido: %d", checksum % 256, raw[19]);
  if (checksum % 256 != raw[19]) {
    RCLCPP_WARN(rclcpp::get_logger("FireBotHardwareInterface"), "Checksum inválido");
    return;
  }

  // Funciones auxiliares para leer valores de 2 bytes (little endian)
  auto read_uint16 = [](const uint8_t* ptr) -> uint16_t {
    return static_cast<uint16_t>(ptr[0] | (ptr[1] << 8));
  };

  auto read_int16 = [](const uint8_t* ptr) -> int16_t {
    return static_cast<int16_t>(ptr[0] | (ptr[1] << 8));
  };

  // Leer sensores IR (3 valores uint16)
  ir_states_[0] = read_uint16(&raw[1]);
  ir_states_[1] = read_uint16(&raw[3]);
  ir_states_[2] = read_uint16(&raw[5]);

  // Leer IMU (6 valores int16), escalar a double (ejemplo: /1000.0)
  for (int i = 0; i < 6; ++i) {
    imu_states_[i] = static_cast<double>(read_int16(&raw[7 + i * 2]))/1000;
  }

  RCLCPP_INFO(rclcpp::get_logger("FireBotHardwareInterface"), "Datos sensores actualizados");
}

bool FireBotHardwareInterface::readAndParsePacket() {
  const size_t PACKET_SIZE = 21;
  const uint8_t HEADER = 'A';
  const uint8_t END1 = '#';
  const uint8_t END2 = '\n';

  std::string buffer;
  buffer.reserve(PACKET_SIZE);

  // Paso 1: Buscar header 'A'
  while (true) {
    char c;
    try {
      c = serial_->read_fixed(1)[0];  // Leer 1 byte
    } catch (const std::exception &e) {
      RCLCPP_WARN(rclcpp::get_logger("FireBotHardwareInterface"), "Error lectura serial: %s", e.what());
      return false;
    }
    if (c == HEADER) {
      buffer.push_back(c);
      break;
    }
  }

  // Paso 2: Leer los siguientes bytes para completar el paquete
  try {
    std::string rest = serial_->read_fixed(PACKET_SIZE - 1);
    if (rest.size() != PACKET_SIZE - 1) {
      RCLCPP_WARN(rclcpp::get_logger("FireBotHardwareInterface"), "Paquete incompleto");
      return false;
    }
    buffer += rest;
  } catch (const std::exception &e) {
    RCLCPP_WARN(rclcpp::get_logger("FireBotHardwareInterface"), "Error lectura serial: %s", e.what());
    return false;
  }

  // Paso 3: Validar paquete (tamaño, header, end)
  if (buffer.size() != PACKET_SIZE || buffer[0] != HEADER ||
      (buffer[PACKET_SIZE-1] != END1 && buffer[PACKET_SIZE-1] != END2)) {
    RCLCPP_WARN(rclcpp::get_logger("FireBotHardwareInterface"), "Paquete inválido (mal formato)");
    return false;
  }

  // Paso 4: Parsear el paquete (usa tu función parseSensorData)
  parseSensorData(buffer);

  return true;
}






} // namespace robot_bombero_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robot_bombero_hardware::FireBotHardwareInterface, hardware_interface::SystemInterface)
