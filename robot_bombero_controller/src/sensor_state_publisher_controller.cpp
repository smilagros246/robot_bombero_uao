#include "robot_bombero_controller/sensor_state_publisher_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace robot_bombero_controller
{

SensorStatePublisherController::SensorStatePublisherController()
: controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn SensorStatePublisherController::on_init()
{
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SensorStatePublisherController::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration SensorStatePublisherController::state_interface_configuration() const
{
  std::vector<std::string> names;

  // IMU: 6 interfaces
  names.push_back("imu_sensor/angular_velocity.x");
  names.push_back("imu_sensor/angular_velocity.y");
  names.push_back("imu_sensor/angular_velocity.z");
  names.push_back("imu_sensor/linear_acceleration.x");
  names.push_back("imu_sensor/linear_acceleration.y");
  names.push_back("imu_sensor/linear_acceleration.z");

  // IRs
  names.push_back("sharp_front/analog_input");
  names.push_back("sharp_left/analog_input");
  names.push_back("sharp_right/analog_input");

  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::CallbackReturn SensorStatePublisherController::on_configure(const rclcpp_lifecycle::State &)
{
  imu_pub_ = get_node()->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
  ir_pub_ = get_node()->create_publisher<std_msgs::msg::Float32MultiArray>("ir/data", 10);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SensorStatePublisherController::on_activate(const rclcpp_lifecycle::State &)
{
  imu_pub_->on_activate();
  ir_pub_->on_activate();

  imu_states_.clear();
  ir_states_.clear();

  for (auto &interface : state_interfaces_)
  {
    if (interface.get_name().rfind("imu_sensor/", 0) == 0)
    {
      imu_states_.emplace_back(interface);
    }
    else if (interface.get_name().rfind("sharp_", 0) == 0)
    {
      ir_states_.emplace_back(interface);
    }
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SensorStatePublisherController::on_deactivate(const rclcpp_lifecycle::State &)
{
  imu_pub_->on_deactivate();
  ir_pub_->on_deactivate();
  release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type SensorStatePublisherController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Publicar IMU
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.angular_velocity.x = imu_states_[3].get().get_value();
  imu_msg.angular_velocity.y = imu_states_[4].get().get_value();
  imu_msg.angular_velocity.z = imu_states_[5].get().get_value();
  imu_msg.linear_acceleration.x = imu_states_[0].get().get_value();
  imu_msg.linear_acceleration.y = imu_states_[1].get().get_value();
  imu_msg.linear_acceleration.z = imu_states_[2].get().get_value();

  // Debug: imprimir en consola
//   RCLCPP_INFO(get_node()->get_logger(), "IMU angular_velocity: [%.3f, %.3f, %.3f]",
//               imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
//   RCLCPP_INFO(get_node()->get_logger(), "IMU linear_acceleration: [%.3f, %.3f, %.3f]",
//               imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);

  imu_pub_->publish(imu_msg);

  // Publicar IR
  std_msgs::msg::Float32MultiArray ir_msg;
  for (const auto &ir : ir_states_)
  {
    ir_msg.data.push_back(ir.get().get_value());
  }
  ir_pub_->publish(ir_msg);

  return controller_interface::return_type::OK;
}

}  // namespace robot_bombero_controller

PLUGINLIB_EXPORT_CLASS(robot_bombero_controller::SensorStatePublisherController,
                       controller_interface::ControllerInterface)
