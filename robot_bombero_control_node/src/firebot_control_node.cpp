#include "robot_bombero_control_node/firebot_control_node.hpp"
#include "std_msgs/msg/float64.hpp"  // Para el mensaje Float64 de la bomba

namespace robot_bombero_control_node
{

FirebotControlNode::FirebotControlNode(std::vector<double>& ir_values, std::vector<double>& imu_values)
: Node("firebot_control_node"),
  ir_values_(ir_values),
  imu_values_(imu_values)
{
    RCLCPP_INFO(this->get_logger(), "Inicializando nodo de control del robot bombero");

    mecanum_controller_ = std::make_shared<robot_bombero_controller::MecanumController>();
    arm_controller_ = std::make_shared<arm_servo_controller::ArmServoController>();

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&FirebotControlNode::cmdVelCallback, this, std::placeholders::_1));

    arm_pos_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/joint_positions", 10,
        std::bind(&FirebotControlNode::armPositionCallback, this, std::placeholders::_1));

    water_pump_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/water_pump_cmd", 10,
        std::bind(&FirebotControlNode::waterPumpCmdCallback, this, std::placeholders::_1));

    ir_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ir_sensors", 10);
    imu_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/imu_data", 10);

    // Timer para publicar datos cada 100ms
    sensor_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&FirebotControlNode::publishSensorData, this));
}

void FirebotControlNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    mecanum_controller_->set_cmd_vel(msg->linear.x, msg->linear.y, msg->angular.z);
    RCLCPP_DEBUG(this->get_logger(), "Recibido cmd_vel: x=%.2f y=%.2f yaw=%.2f", msg->linear.x, msg->linear.y, msg->angular.z);
}

void FirebotControlNode::armPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 4)
    {
        std::vector<double> targets = {
        msg->data[0],
        msg->data[1],
        msg->data[2],
        msg->data[3]
    };
    arm_controller_->set_joint_position_targets(targets);
        RCLCPP_DEBUG(this->get_logger(), "Recibidas posiciones del brazo");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "El mensaje de posiciones del brazo tiene menos de 4 elementos.");
    }
}



// Callback para recibir el comando de la bomba
void FirebotControlNode::waterPumpCmdCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    // Llamamos a la función del controlador para almacenar el valor recibido
    arm_controller_->set_water_pump_effort(msg->data);

    // Imprimir el valor recibido para verificar
    RCLCPP_DEBUG(this->get_logger(), "Comando de bomba recibido: %.2f", msg->data);
}

void FirebotControlNode::publishSensorData()
{
    std_msgs::msg::Float64MultiArray ir_msg;
    ir_msg.data = ir_values_;
    ir_pub_->publish(ir_msg);

    std_msgs::msg::Float64MultiArray imu_msg;
    imu_msg.data = imu_values_;
    imu_pub_->publish(imu_msg);
}


} // namespace robot_bombero_control_node

// Main
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto firebot_hw = std::make_shared<firebot_hardware::FireBotHardwareInterface>();
    auto node = std::make_shared<robot_bombero_control_node::FirebotControlNode>(
        firebot_hw->getIRValues(), firebot_hw->getIMUValues());

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
