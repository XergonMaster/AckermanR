#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm> // Para std::min_element
#include <limits>    // Para std::numeric_limits
#include <chrono>

class EmergencyBrakeNode : public rclcpp::Node
{
public:
    EmergencyBrakeNode() : Node("emergency_brake_node")
    {
        // Configurar la calidad de servicio (QoS) para "Best Effort"
        rclcpp::QoS qos(10); // Cambia el número 10 según tus necesidades
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        // Suscribirse al mensaje de escaneo láser
        laser_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&EmergencyBrakeNode::laserCallback, this, std::placeholders::_1));

        // Suscribirse al comando de velocidad (Twist)
        cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_out", qos,
            std::bind(&EmergencyBrakeNode::cmdVelCallback, this, std::placeholders::_1));

        // Publicar mensajes Twist en el tópico cmd_EB
        cmd_eb_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_EB", 10);

        // Configurar la duración mínima entre intentos de recuperación (por ejemplo, 5 segundos)
        min_recovery_interval_ = std::chrono::seconds(5);
        last_recovery_time_ = std::chrono::steady_clock::now();

        // Configurar la duración de tiempo para enviar el comando de recuperación (por ejemplo, 2 segundos)
        recovery_duration_ = std::chrono::seconds(2);
        recovery_start_time_ = std::chrono::steady_clock::now();
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Guardar la velocidad lineal en una variable miembro
        current_linear_velocity_ = msg->linear.x;

        // Lógica para verificar la odometría y decidir si se activa el freno de emergencia
        // Puedes usar current_linear_velocity_ para acceder a la velocidad lineal
        // y tomar decisiones basadas en esa velocidad.
        // ...
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
                // RCLCPP_INFO(this->get_logger(), "rate: %.2f , vel: %.2f ", range_rate, current_linear_velocity_);
                double TTC = 0.0;

                if (range_rate != 0.0 && current_linear_velocity_ > 0)
                {
                    TTC = std::abs(min_distance / range_rate);
                }
                else
                {
                    TTC = std::numeric_limits<double>::infinity();
                }

                // RCLCPP_INFO(this->get_logger(), "Dis: %.2f m, TTC: %.2f s", min_distance, TTC);

                if (TTC < 2.0)
                {
                    RCLCPP_WARN(this->get_logger(), "Obstacle detected in front! Distance: %.2f m, TTC: %.2f s", min_distance, TTC);

                    // Verificar si ha pasado suficiente tiempo desde el último intento de recuperación
                    auto current_time = std::chrono::steady_clock::now();
                    auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_recovery_time_);

                    if (elapsed_time >= min_recovery_interval_)
                    {
                        // Iniciar un intento de recuperación
                        initiateRecovery();
                    }
                }
            }
        }
    }

    // Función para iniciar un intento de recuperación
    // Función para iniciar un intento de recuperación
    void initiateRecovery()
    {
        RCLCPP_INFO(this->get_logger(), "Initiating recovery...");

        // Configurar el mensaje de recuperación (por ejemplo, retroceder)
        geometry_msgs::msg::Twist recovery_msg;
        recovery_msg.linear.x = -0.5;
        recovery_msg.linear.y = 0.0;
        recovery_msg.angular.z = 0.0;

        // Obtener el tiempo actual
        auto current_time = std::chrono::steady_clock::now();

        // Enviar el comando de recuperación durante el período de recuperación
        while (current_time - recovery_start_time_ < recovery_duration_)
        {
            RCLCPP_INFO(this->get_logger(), "recovery...");

            cmd_eb_publisher_->publish(recovery_msg);

            // Actualizar el tiempo actual
            current_time = std::chrono::steady_clock::now();
        }

        // Actualizar el tiempo del último intento de recuperación
        last_recovery_time_ = std::chrono::steady_clock::now();
    }

    // Función para obtener un subconjunto de un vector
    std::vector<float> obtenerRangoVector(const std::vector<float> &vector, int inicio, int fin)
    {
        int longitud = vector.size();

        // Ajustar índices negativos
        if (inicio < 0)
        {
            inicio = longitud + inicio;
        }
        if (fin < 0)
        {
            fin = longitud + fin;
        }

        // Asegurarse de que los índices estén en el rango válido
        inicio = inicio < 0 ? 0 : (inicio > longitud - 1 ? longitud - 1 : inicio);
        fin = fin < 0 ? 0 : (fin > longitud - 1 ? longitud - 1 : fin);

        // Crear el nuevo vector y copiar los elementos
        std::vector<float> resultado;
        if (inicio <= fin)
        {
            for (int i = inicio; i <= fin; i++)
            {
                resultado.push_back(vector[i]);
            }
        }
        else
        {
            for (int i = inicio; i < longitud; i++)
            {
                resultado.push_back(vector[i]);
            }
            for (int i = 0; i <= fin; i++)
            {
                resultado.push_back(vector[i]);
            }
        }

        return resultado;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_eb_publisher_;

    bool emergency_detected = false;                            // Variable para controlar si se activ
    double current_linear_velocity_ = 0.0;                      // Variable para almacenar la velocidad lineal actual
    std::chrono::seconds min_recovery_interval_;                // Intervalo mínimo entre intentos de recuperación
    std::chrono::steady_clock::time_point last_recovery_time_;  // Tiempo del último intento de recuperación
    std::chrono::seconds recovery_duration_;                    // Duración del envío constante del comando de recuperación
    std::chrono::steady_clock::time_point recovery_start_time_; // Tiempo de inicio del envío constante del comando de recuperación
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EmergencyBrakeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}