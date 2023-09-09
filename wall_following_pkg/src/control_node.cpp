#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class ControlNode : public rclcpp::Node
{
public:
    ControlNode() : Node("control_node"), last_error_(0), theta_ant_(0)
    {

        this->declare_parameter("VEL_X", 0.3);
        this->declare_parameter("MAX_THETA", 0.8);
        this->declare_parameter("Kp", 0.8);  // Valor por defecto
        this->declare_parameter("Kd", 3.212);  // Valor por defecto

        VEL_X = this->get_parameter("VEL_X").as_double();
        MAX_TETHA = this->get_parameter("MAX_THETA").as_double();
        Kp = this->get_parameter("Kp").as_double();
        Kd = this->get_parameter("Kd").as_double();

        subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/error", 10, std::bind(&ControlNode::error_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_ctrl", 10);
    }

private:
    void error_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {

        
        double dt = this->get_clock()->now().seconds() - last_time_.seconds();

        double de = msg->data - last_error_;
        double theta_d = Kp * msg->data + Kd * de / dt;
        
        theta_ant_ = theta_ant_ - theta_d;
        if(abs(theta_ant_)>MAX_TETHA){
            theta_ant_=theta_ant_/abs(theta_ant_)*MAX_TETHA;
        }
        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = -theta_ant_;
        cmd.linear.x=VEL_X ;
        publisher_->publish(cmd);

        last_error_ = msg->data;
        last_time_ = this->get_clock()->now();
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    double last_error_;
    double theta_ant_;
    double VEL_X=0.3;
    double MAX_TETHA=0.8;
    double Kp = 0.8; // Adjust as needed
    double Kd = 3.212; // Adjust as needed
    rclcpp::Time last_time_ = this->get_clock()->now();
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}