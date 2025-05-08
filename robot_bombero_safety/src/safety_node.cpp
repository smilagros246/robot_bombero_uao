#include "robot_bombero_safety/safety_node.hpp"

SafetyNode::SafetyNode() : Node("safety_node")
{
    // Publicadores
    ir_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ir_sensors", 10);
    imu_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/imu_data", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    safety_status_pub_ = this->create_publisher<std_msgs::msg::String>("/safety_status", 10);

    // Suscripciones
    cmd_vel_raw_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_raw", 10, std::bind(&SafetyNode::cmd_vel_raw_callback, this, std::placeholders::_1));

    // Temporizador para revisar la seguridad
    safety_timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&SafetyNode::safety_check, this));
}

void SafetyNode::cmd_vel_raw_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Guardar el comando de movimiento
    cmd_vel_raw_ = *msg;
}

void SafetyNode::safety_check()
{
    // Suponemos que aquí obtienes los datos reales de los sensores
    std_msgs::msg::Float64MultiArray ir_data;
    std_msgs::msg::Float64MultiArray imu_data;

    // Publicar los datos de IR y IMU
    ir_pub_->publish(ir_data);
    imu_pub_->publish(imu_data);

    // Seguridad: Si detecta un riesgo
    if (detect_obstacle(ir_data) || detect_tilt(imu_data))
    {
        // Detener el robot
        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);

        // Publicar un mensaje de alerta
        std_msgs::msg::String alert_msg;
        alert_msg.data = "Obstacle detected or IMU alert!";
        safety_status_pub_->publish(alert_msg);
    }
    else
    {
        // Todo está seguro, reenviar el comando original
        cmd_vel_pub_->publish(cmd_vel_raw_);
        std_msgs::msg::String safe_msg;
        safe_msg.data = "System Safe";
        safety_status_pub_->publish(safe_msg);
    }
}

bool SafetyNode::detect_obstacle(const std_msgs::msg::Float64MultiArray &ir_data)
{
    // Umbral de distancia para detectar obstáculos en raw
    double threshold_raw = 500; // Ajusta según la especificación de tus sensores IR

    // Recorremos los datos de los sensores IR (suponiendo que cada valor es una distancia)
    for (size_t i = 0; i < ir_data.data.size(); ++i)
    {
        // Si alguno de los valores de IR es menor que el umbral, hay un obstáculo cerca
        if (ir_data.data[i] < threshold_raw)
        {
            RCLCPP_WARN(this->get_logger(), "Obstacle detected! Sensor %zu value: %f", i, ir_data.data[i]);
            return true;  // Objeto detectado
        }
    }

    // Si no hay obstáculos detectados
    return false;
}
bool SafetyNode::detect_tilt(const std_msgs::msg::Float64MultiArray &imu_data)
{
    // Verificar que el mensaje contenga al menos 3 valores
    if (imu_data.data.size() < 3)
    {
        RCLCPP_WARN(this->get_logger(), "IMU data has insufficient size: %zu", imu_data.data.size());
        return false;
    }

    double ax = imu_data.data[0];
    double ay = imu_data.data[1];
    double az = imu_data.data[2];

    double a_mag = std::sqrt(ax*ax + ay*ay + az*az);

    if (a_mag < 1e-2)
    {
        RCLCPP_WARN(this->get_logger(), "IMU acceleration magnitude too small, possibly invalid.");
        return false;
    }

    // Ángulo entre el vector de aceleración y el eje Z
    double tilt_angle = std::acos(az / a_mag) * (180.0 / M_PI);

    double tilt_threshold = 45.0;

    if (tilt_angle > tilt_threshold)
    {
        RCLCPP_WARN(this->get_logger(), "Tilt detected! Angle from vertical: %f°", tilt_angle);
        return true;
    }

    return false;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}
