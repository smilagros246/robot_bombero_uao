#include "robot_bombero_controller/mecanum_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;
namespace robot_bombero_controller
{

MecanumController::MecanumController()
: controller_interface::ControllerInterface()
{}

controller_interface::CallbackReturn MecanumController::on_init()
{
  RCLCPP_INFO(get_node()->get_logger(), "on_init: Inicializando controlador Mecanum...");

  joint_names_ = auto_declare<std::vector<std::string>>("joints", {
    "link_fl_wheel_joint",
    "link_fr_wheel_joint",
    "link_rl_wheel_joint",
    "link_rr_wheel_joint"
  });

  command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", {
    hardware_interface::HW_IF_VELOCITY
  });

  state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", {
    hardware_interface::HW_IF_VELOCITY
  });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MecanumController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : command_interface_types_)
    {
      config.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return config;
}

controller_interface::InterfaceConfiguration MecanumController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf = {
    controller_interface::interface_configuration_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : state_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}


controller_interface::CallbackReturn MecanumController::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  RCLCPP_INFO(node->get_logger(), "on_configure: Configurando controlador...");

  // Leer parámetros si están disponibles
  node->get_parameter_or("wheel_radius", wheel_radius_, wheel_radius_);
  node->get_parameter_or("wheel_base_x", wheel_base_x_, wheel_base_x_);
  node->get_parameter_or("wheel_base_y", wheel_base_y_, wheel_base_y_);

  // Suscripción a cmd_vel
  cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::SystemDefaultsQoS(),
    std::bind(&MecanumController::cmdVelCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(), "on_configure: Parámetros leídos y suscripción creada.");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumController::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_activate: Activando controlador...");

  // Limpiar vectores
  wheel_command_interfaces_.clear();
  wheel_state_interfaces_.clear();

  for (auto & interface : command_interfaces_) {
   wheel_command_interfaces_.push_back(std::ref(interface));
  }

  for (auto & interface : state_interfaces_) {
   wheel_state_interfaces_.push_back(std::ref(interface));
  }

  cmd_vel_buffer_.writeFromNonRT(geometry_msgs::msg::Twist());

  odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());
  last_time_ = get_node()->now();


  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn MecanumController::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_deactivate: Desactivando controlador...");

  // Limpieza de suscripción
  cmd_vel_sub_.reset();

  // Detener motores
  for (auto & interface : wheel_command_interfaces_) {
    interface.get().set_value(0.0);  // usando .get() por si usas std::ref
  }

  // Liberar interfaces de comando para permitir que otros controladores las usen
  release_interfaces();

  return controller_interface::CallbackReturn::SUCCESS;
}



void MecanumController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_buffer_.writeFromNonRT(*msg);
}

controller_interface::return_type MecanumController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Leer el último comando cmd_vel recibido (de forma segura en tiempo real)
  const auto cmd_vel = *(cmd_vel_buffer_.readFromRT());

  // Calcular velocidades individuales para cada rueda usando cinemática inversa
  double v1, v2, v3, v4;
  mecanumInverseKinematics(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, v1, v2, v3, v4);

  // Verificar que tengamos interfaces para las 4 ruedas
  if (wheel_command_interfaces_.size() == 4)
  {
    wheel_command_interfaces_[0].get().set_value(v1);
    wheel_command_interfaces_[1].get().set_value(v2);
    wheel_command_interfaces_[2].get().set_value(v3);
    wheel_command_interfaces_[3].get().set_value(v4);
  }
  else
  {
    RCLCPP_WARN(get_node()->get_logger(), "No hay suficientes interfaces de comando para las ruedas.");
  }

  //Odometria
  const double dt = (get_node()->now() - last_time_).seconds();

  // Por ahora puedes usar los cmd_vel como dummy
  const double vx = cmd_vel.linear.x;
  const double vy = cmd_vel.linear.y;
  const double vth = cmd_vel.angular.z;

  // Integración de posición (en coordenadas locales)
  const double delta_x = (vx * std::cos(theta_) - vy * std::sin(theta_)) * dt;
  const double delta_y = (vx * std::sin(theta_) + vy * std::cos(theta_)) * dt;
  const double delta_theta = vth * dt;

  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_theta;

  // Publicar odometría
  geometry_msgs::msg::Quaternion odom_quat;
  odom_quat.z = std::sin(theta_ * 0.5);
  odom_quat.w = std::cos(theta_ * 0.5);

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = get_node()->now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = vth;

  odom_pub_->publish(odom_msg);

  // Publicar transformada
  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.stamp = get_node()->now();
  odom_tf.header.frame_id = "odom";
  odom_tf.child_frame_id = "base_footprint";

  odom_tf.transform.translation.x = x_;
  odom_tf.transform.translation.y = y_;
  odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation = odom_quat;

  tf_broadcaster_->sendTransform(odom_tf);

  last_time_ = get_node()->now();


  return controller_interface::return_type::OK;
}


void MecanumController::mecanumInverseKinematics(
  double vx, double vy, double omega,
  double &v1, double &v2, double &v3, double &v4)
{
  // Fórmulas estándar para mecanum wheels
  // v_wheel = (1 / r) * (vx ± vy ± (L + W) * omega)
  // Ajusta signos según tu convención de ejes y orden de ruedas

  double L = wheel_base_x_;
  double W = wheel_base_y_;
  double r = wheel_radius_;

  v1 = (1.0 / r) * (vx - vy - (L + W) * omega);  // Front left
  v2 = (1.0 / r) * (vx + vy + (L + W) * omega);  // Front right
  v3 = (1.0 / r) * (vx + vy - (L + W) * omega);  // Rear left
  v4 = (1.0 / r) * (vx - vy + (L + W) * omega);  // Rear right
}

}  // namespace mecanum_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_bombero_controller::MecanumController, controller_interface::ControllerInterface)
