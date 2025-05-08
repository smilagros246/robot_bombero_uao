#ifndef ROBOT_BOMBERO_SAFETY__SAFETY_NODE_HPP_
#define ROBOT_BOMBERO_SAFETY__SAFETY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

class SafetyNode : public rclcpp::Node
{
public:
    SafetyNode();

private:
    void cmd_vel_raw_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void safety_check();

    bool detect_obstacle(const std_msgs::msg::Float64MultiArray &ir_data);
    bool detect_tilt(const std_msgs::msg::Float64MultiArray &imu_data);

    // Publicadores
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ir_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_status_pub_;

    // Suscripciones
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_raw_sub_;

    // Variables temporales
    geometry_msgs::msg::Twist cmd_vel_raw_;
    rclcpp::TimerBase::SharedPtr safety_timer_;
};

#endif  // ROBOT_BOMBERO_SAFETY__SAFETY_NODE_HPP_
