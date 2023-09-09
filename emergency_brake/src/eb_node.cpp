#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm> // Para std::min_element
#include <limits>    // Para std::numeric_limits

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
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Guardar la velocidad lineal en una variable miembro
        current_linear_velocity_ = msg->twist.twist.linear.x;

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
                    geometry_msgs::msg::Twist stop_msg;
                    stop_msg.linear.x = 0.0;
                    stop_msg.linear.y = 0.0;
                    stop_msg.angular.z = 0.0;
                    cmd_eb_publisher_->publish(stop_msg);
                }
            }
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_eb_publisher_;

    bool emergency_detected = false;           // Variable para controlar si se activa el freno de emergencia
    double current_linear_velocity_ = 0.0;     // Variable para almacenar la velocidad lineal actual
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EmergencyBrakeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
