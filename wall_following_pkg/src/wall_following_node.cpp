#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>

class WallFollowing : public rclcpp::Node
{
public:
    WallFollowing() : Node("wall_following_node")
    {

        this->declare_parameter("DIS_WALL", 2.0);
        this->declare_parameter("SAFETY_DISTANCE", 2.0);
        DIS_WALL = this->get_parameter("DIS_WALL").as_double();
        SAFETY_DISTANCE = this->get_parameter("SAFETY_DISTANCE").as_double();

        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallFollowing::scan_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/error", 10);
        publisher_dist = this->create_publisher<std_msgs::msg::Float32>("/dist_wall", 10);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {   
        double theta = M_PI / 18 * 3;  // 30 degrees in radians
        // double right_beam_angle = 3/4 * M_PI ;
        // double delta_beam_angle = right_beam_angle - theta ; 
        
        // int right_beam_index = round((right_beam_angle - msg->angle_min) / msg->angle_increment);
        // int delta_beam_index =  round((delta_beam_angle - msg->angle_min) / msg->angle_increment);

        double b = msg->ranges[300];
        double a = msg->ranges[270];

        
        double alpha = atan((a * cos(theta) - b) / (a * sin(theta)));
        double AB = b * cos(alpha);
        double AC = 0.05;
        double CD = AB + AC * sin(alpha);

        
        double error = DIS_WALL - CD;

        double front_distance = msg->ranges[0];
        double rigth_distance = msg->ranges[270];
        double back_distance = msg->ranges[180];
        double  left_distance = msg->ranges[90];
        RCLCPP_INFO(this->get_logger(), "f: %.2f r: %.2f b: %.2f l: %.2f e: %.2f",front_distance,
        rigth_distance,back_distance,left_distance,error);
        if (front_distance < SAFETY_DISTANCE)
        {
            RCLCPP_INFO(this->get_logger(), "Obstacle detected in front!");
            // // You can implement logic here to make the robot turn or back up
            // // For example, stop and turn left
            error = 10.0; // Set the error to indicate a turn to the left
        }
        std_msgs::msg::Float32 error_msg;
        error_msg.data = error;
        publisher_->publish(error_msg);

        std_msgs::msg::Float32 dist_wall_msg;
        dist_wall_msg.data = CD;
        publisher_dist->publish(dist_wall_msg);
    }
    double DIS_WALL=2.0;
    double SAFETY_DISTANCE=2.0;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_dist;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollowing>());
    rclcpp::shutdown();
    return 0;
}