#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"


#include "robot_bombero_controller/mecanum_controller.hpp"
#include "robot_bombero_controller/arm_servo_controller.hpp"
#include "robot_bombero_hardware/firebot_hardware_interface.hpp"


namespace robot_bombero_control_node
{

class FirebotControlNode : public rclcpp::Node
{
public:
    FirebotControlNode(std::vector<double>& ir_values, std::vector<double>& imu_values);
    void publishSensorData();

private:
    // Subscripciones
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr water_pump_cmd_sub_;

    // Controladores (punteros simulados — en realidad ros2_control los instanciaría en tiempo de ejecución)
    std::shared_ptr<robot_bombero_controller::MecanumController> mecanum_controller_;
    std::shared_ptr<arm_servo_controller::ArmServoController> arm_controller_;

    // Callbacks
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void armPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void waterPumpCmdCallback(const std_msgs::msg::Float64::SharedPtr msg);

    std::vector<double>& ir_values_;
    std::vector<double>& imu_values_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ir_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr sensor_timer_;
};

}  // namespace robot_bombero_control_node
