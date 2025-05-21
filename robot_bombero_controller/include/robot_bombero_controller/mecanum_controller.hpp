#ifndef MECANUM_CONTROLLER_HPP_
#define MECANUM_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <realtime_tools/realtime_buffer.hpp>
#include "geometry_msgs/msg/twist.hpp"

namespace robot_bombero_controller
{

class MecanumController : public controller_interface::ControllerInterface
{
public:
  MecanumController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_ ;

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> cmd_vel_buffer_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> wheel_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    wheel_state_interfaces_;

  // Parámetros del robot mecanum
  double wheel_radius_{0.0325};  // metros
  double wheel_base_x_{0.15};   // distancia eje x Distancia entre ejes delanteros y traseros (de front-left a rear-left).
  double wheel_base_y_{0.13};  // distancia eje y pendiete medir Distancia entre las ruedas de un mismo eje (de left a right).

  //Odometría 
  // Broadcaster y mensaje de odometría
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // Estado acumulado
  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;

  // Último timestamp
  rclcpp::Time last_time_;

  // Método para la cinemática inversa mecanum
  void mecanumInverseKinematics(
    double vx, double vy, double omega,
    double &v1, double &v2, double &v3, double &v4);
};


}  // namespace robot_bombero_controller

#endif  // MECANUM_CONTROLLER_HPP_
