#ifndef MECANUM_CONTROLLER_HPP
#define MECANUM_CONTROLLER_HPP

#include <memory>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace robot_bombero_controller
{

class MecanumController : public controller_interface::ControllerInterface
{
public:
    MecanumController();
    ~MecanumController() override = default;

    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    void set_cmd_vel(double x, double y, double yaw);

private:
    std::vector<hardware_interface::LoanedCommandInterface> joint_handles_;
    double cmd_vel_x_, cmd_vel_y_, cmd_vel_yaw_;
    std::vector<std::string> wheel_names_;
};

} // namespace robot_bombero_controller

#endif  // MECANUM_CONTROLLER_HPP
