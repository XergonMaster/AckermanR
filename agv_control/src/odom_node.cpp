#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <Eigen/Dense>

class OdomNode : public rclcpp::Node
{
public:
    OdomNode() : Node("odom_node")
    {
        motor_velocities_subs_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "motor_velocities", 10,
            std::bind(&OdomNode::onMotorVelocitiesMessage, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        RCLCPP_INFO(this->get_logger(), "odom_node has started");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motor_velocities_subs_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    Eigen::Matrix2f rot_z(float psi)
    {
        Eigen::Matrix2f R;
        R << cos(psi), -sin(psi),
             sin(psi),  cos(psi);
        return R;
    }

    Eigen::Vector3f fn_wi(float a, float phi, float theta, float dx, float dy)
    {
        Eigen::RowVector2f c1(1/a, 1/a * tan(phi));
        Eigen::Matrix2f c2 = rot_z(theta);
        Eigen::Matrix<float, 2, 3> c3;
        c3 << 1, 0, -dy,
              0, 1,  dx;

        return c1 * c2 * c3;
    }

    void onMotorVelocitiesMessage(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 4) {
            RCLCPP_ERROR(this->get_logger(), "Received an unexpected number of motor velocities");
            return;
        }

        // Define los parámetros
        Eigen::Vector4f phi(-M_PI/4, M_PI/4, -M_PI/4, M_PI/4);
        float a = 0.0375;
        Eigen::Vector4f ai(a, a, a, a);
        Eigen::Vector4f theta(0, 0, 0, 0);
        float l = 0.17;
        float d = 0.15;
        Eigen::Vector4f dx(l, -l, -l, l);
        Eigen::Vector4f dy(d, d, -d, -d);

        Eigen::Matrix<float, 4, 3> omega_i;

        // Calcular omega_i
        for (int i = 0; i < 4; i++) {
            omega_i.row(i) = fn_wi(ai(i), phi(i), theta(i), dx(i), dy(i));
        }

        // TODO: Calculate W here using Eigen functions.
        Eigen::Matrix<float, 3, 4> W = (omega_i.transpose() * omega_i).inverse() * omega_i.transpose();

        // Log the matrix W
        for (int i = 0; i < 4; i++) {
            std::string row_str;
            for (int j = 0; j < 3; j++) {
                row_str += std::to_string(omega_i(i, j)) + " ";  // Replace omega_i with W once calculated
            }
            RCLCPP_INFO(this->get_logger(), row_str.c_str());
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto odom_node = std::make_shared<OdomNode>();
    rclcpp::spin(odom_node);
    rclcpp::shutdown();
    return 0;
}
