#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

class LidarProcessingNode : public rclcpp::Node
{
public:
    LidarProcessingNode()
    : Node("lidar_processing_node"),
      velocity_x_(0.0)
    {
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&LidarProcessingNode::lidar_callback, this, std::placeholders::_1));

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&LidarProcessingNode::odom_callback, this, std::placeholders::_1));
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr &msg)
    {
        float min_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
        if (min_distance < 0.5)
        {
            RCLCPP_WARN(this->get_logger(), "Obstacle detected! Distance: %.2f m", min_distance);

            // Calcula el Tiempo para la Colisión (TTC)
            if(velocity_x_ != 0.0)  // Evita división por cero
            {
                float TTC = min_distance / -velocity_x_;
                RCLCPP_INFO(this->get_logger(), "Time to Collision (TTC): %.2f s", TTC);

                if (TTC < 3.0)
                {
                    geometry_msgs::msg::Twist stop_msg;
                    stop_msg.linear.x = 0.0;
                    stop_msg.linear.y = 0.0;
                    stop_msg.linear.z = 0.0;
                    stop_msg.angular.x = 0.0;
                    stop_msg.angular.y = 0.0;
                    stop_msg.angular.z = 0.0;
                    cmd_vel_publisher_->publish(stop_msg);
                }
            }
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr &msg)
    {
        velocity_x_ = msg->twist.twist.linear.x;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    double velocity_x_;
};