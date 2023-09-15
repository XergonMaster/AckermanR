#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm> // Para std::min_element
#include <limits>    // Para std::numeric_limits
#include <chrono>    // Para medir el tiempo

class EmergencyBrakeNode : public rclcpp::Node
{
public:
    EmergencyBrakeNode() : Node("emergency_brake_node")
    {
        // Configurar la calidad de servicio (QoS) para "Best Effort"
        rclcpp::QoS qos(10); // Cambia el número 10 según tus necesidades
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "/car/odom", qos,
            std::bind(&EmergencyBrakeNode::odomCallback, this, std::placeholders::_1));

        // Suscribirse al mensaje de escaneo láser
        laser_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&EmergencyBrakeNode::laserCallback, this, std::placeholders::_1));

        // Publicar mensajes Twist en el tópico cmd_EB
        cmd_eb_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_EB", 10);

        // Inicializar variables de recuperación
        recovery_attempts_ = 0;
        recovery_duration_ = std::chrono::seconds(5); // Duración de cada intento de recuperación
        last_recovery_time_ = std::chrono::steady_clock::now();
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Guardar la velocidad lineal en una variable miembro
        current_linear_velocity_ = msg->twist.twist.linear.x;
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        double angle_min_front = -7 * (M_PI / 180);
        double angle_max_front = 7 * (M_PI / 180);

        int index_min = round((angle_min_front - msg->angle_min) / msg->angle_increment);
        int index_max = round((angle_max_front - msg->angle_min) / msg->angle_increment);

        auto ranges = msg->ranges;

        std::vector<float> resultado = obtenerRangoVector(ranges, index_min, index_max);
        int nuevaLongitud = resultado.size();

        // Inicializar la distancia mínima y el ángulo correspondiente con valores predeterminados
        double min_distance = std::numeric_limits<double>::infinity();
        double angle_min_value = 0.0;

        // Verificar que nuevaLongitud sea mayor que 0
        if (nuevaLongitud > 0)
        {
            // Encontrar la distancia mínima y el ángulo correspondiente
            for (int i = 0; i < nuevaLongitud; i++)
            {
                if (resultado[i] < min_distance)
                {
                    min_distance = resultado[i];
                    angle_min_value = msg->angle_min + (index_min + i) * msg->angle_increment;
                }
            }

            if (std::isinf(min_distance))
            {
                // La distancia mínima es infinita, no hay obstáculos detectados
                RCLCPP_INFO(this->get_logger(), "No obstacles detected in front.");
            }
            else
            {
                double range_rate = current_linear_velocity_ * std::cos(angle_min_value);
                double TTC = 0.0;

                if (range_rate != 0.0)
                {
                    TTC = std::abs(min_distance / range_rate);
                }
                else
                {
                    TTC = std::numeric_limits<double>::infinity();
                }

                if (TTC < 3.0)
                {
                    RCLCPP_WARN(this->get_logger(), "Obstacle detected in front! Distance: %.2f m, TTC: %.2f s", min_distance, TTC);

                    // Verificar si se debe iniciar un intento de recuperación
                    auto current_time = std::chrono::steady_clock::now();
                    if (recovery_attempts_ < max_recovery_attempts_ &&
                        current_time - last_recovery_time_ > recovery_duration_)
                    {
                        initiateRecovery();
                    }
                    else
                    {
                        // Detener el vehículo si no se pueden realizar más intentos de recuperación
                        geometry_msgs::msg::Twist stop_msg;
                        stop_msg.linear.x = 0.0;
                        stop_msg.linear.y = 0.0;
                        stop_msg.angular.z = 0.0;
                        cmd_eb_publisher_->publish(stop_msg);
                    }
                }
            }
        }
    }

    // Función para iniciar un intento de recuperación
    void initiateRecovery()
    {
        RCLCPP_WARN(this->get_logger(), "Initiating recovery attempt %d", recovery_attempts_);
        geometry_msgs::msg::Twist recovery_msg;

        // Hacer que el vehículo retroceda durante 2 segundos (ajusta el tiempo según tus necesidades)
        auto start_time = std::chrono::steady_clock::now();
        auto current_time = start_time;

        while (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() < 2)
        {
            recovery_msg.linear.x = -1.0;  // Velocidad lineal negativa para retroceder
            recovery_msg.linear.y = 0.0;
            recovery_msg.angular.z = 0.0;
            cmd_eb_publisher_->publish(recovery_msg);

            rclcpp::spin_some(node);  // Procesar mensajes para evitar bloqueos
            rclcpp::sleep_for(std::chrono::milliseconds(100));  // Pequeña pausa
            current_time = std::chrono::steady_clock::now();
        }

        // Detener el vehículo después de retroceder
        recovery_msg.linear.x = 0.0;
        cmd_eb_publisher_->publish(recovery_msg);

        // Actualizar el tiempo del último intento de recuperación
        last_recovery_time_ = std::chrono::steady_clock::now();
        recovery_attempts_++;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_eb_publisher_;

    double current_linear_velocity_ = 0.0;           // Variable para almacenar la velocidad lineal actual
    int recovery_attempts_ = 0;                      // Número de intentos de recuperación realizados
    int max_recovery_attempts_ = 5;                  // Número máximo de intentos de recuperación
    std::chrono::steady_clock::duration recovery_duration_; // Duración de cada intento de recuperación
    std::chrono::steady_clock::time_point last_recovery_time_; // Tiempo del último intento de recuperación
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EmergencyBrakeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
