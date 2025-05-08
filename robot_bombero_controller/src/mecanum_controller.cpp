#include "robot_bombero_controller/mecanum_controller.hpp"

#include <rclcpp/logging.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>

namespace robot_bombero_controller
{

MecanumController::MecanumController()
: cmd_vel_x_(0.0), cmd_vel_y_(0.0), cmd_vel_yaw_(0.0)
{}

controller_interface::CallbackReturn MecanumController::on_init()
{
    // Declarar el parámetro dinámico para los nombres de las ruedas
    get_node()->declare_parameter<std::vector<std::string>>("wheels",
        {"wheel_front_left", "wheel_front_right", "wheel_back_left", "wheel_back_right"});

    // Obtener los nombres de las ruedas desde los parámetros
    get_node()->get_parameter("wheels", wheel_names_);

    // Validar el número de ruedas
    if (wheel_names_.size() != 4)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Se esperan 4 ruedas, pero se encontraron %lu", wheel_names_.size());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MecanumController::command_interface_configuration() const
{
    std::vector<std::string> wheel_velocity_interfaces;
    for (const auto &wheel_name : wheel_names_)
    {
        wheel_velocity_interfaces.push_back(wheel_name + "_velocity");
    }

    return {
        controller_interface::interface_configuration_type::INDIVIDUAL,
        wheel_velocity_interfaces};
}

controller_interface::InterfaceConfiguration MecanumController::state_interface_configuration() const
{
    return {
        controller_interface::interface_configuration_type::NONE,
        {}};
}

controller_interface::CallbackReturn MecanumController::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_node()->get_logger(), "Activando controlador de Mecanum");

    auto handle = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&](const auto &interface) {
        return std::any_of(wheel_names_.begin(), wheel_names_.end(), 
                            [&](const std::string &name) {
                                return interface.get_name() == name + "_velocity";
                            });
    });

    for (const auto& name : wheel_names_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Command interface not found for %s", name.c_str());
    }
    joint_handles_.push_back(std::move(*handle));

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumController::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_node()->get_logger(), "Desactivando controlador mecanum");
    joint_handles_.clear();  // opcional: liberar interfaces
    return controller_interface::CallbackReturn::SUCCESS;
}

void MecanumController::set_cmd_vel(double x, double y, double yaw)
{
    cmd_vel_x_ = x;
    cmd_vel_y_ = y;
    cmd_vel_yaw_ = yaw;
}

controller_interface::return_type MecanumController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // Cálculo de velocidades para cada rueda según cinemática Mecanum
    double v1 = cmd_vel_x_ - cmd_vel_y_ - cmd_vel_yaw_;
    double v2 = cmd_vel_x_ + cmd_vel_y_ + cmd_vel_yaw_;
    double v3 = cmd_vel_x_ + cmd_vel_y_ - cmd_vel_yaw_;
    double v4 = cmd_vel_x_ - cmd_vel_y_ + cmd_vel_yaw_;

    double max_speed = 255.0;

    std::vector<double> wheel_velocities = {
        std::clamp(v1 * max_speed, 0.0, max_speed),
        std::clamp(v2 * max_speed, 0.0, max_speed),
        std::clamp(v3 * max_speed, 0.0, max_speed),
        std::clamp(v4 * max_speed, 0.0, max_speed)};

    for (size_t i = 0; i < joint_handles_.size(); ++i)
    {
        joint_handles_[i].set_value(wheel_velocities[i]);
    }

    return controller_interface::return_type::OK;
}

} // namespace robot_bombero_controller

PLUGINLIB_EXPORT_CLASS(robot_bombero_controller::MecanumController, controller_interface::ControllerInterface)
