#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class FollowTheGapNode : public rclcpp::Node
{
public:
    FollowTheGapNode() : Node("follow_the_gap_node")
    {
        this->declare_parameter("VEL_X", 0.3);
        this->declare_parameter("MAX_THETA", 0.8);
        this->declare_parameter("Kp", 0.8);  // Valor por defecto
        this->declare_parameter("Kd", 8.90); // Valor por defecto
        this->declare_parameter("Switch", 1.0);
        this->declare_parameter("Ki", 0.5);
        this->declare_parameter("OUT_RANGE", 240);

        Switch = this->get_parameter("Switch").as_double();
        VEL_X = this->get_parameter("VEL_X").as_double();
        MAX_TETHA = this->get_parameter("MAX_THETA").as_double();
        Kp = this->get_parameter("Kp").as_double();
        Kd = this->get_parameter("Kd").as_double();
        Ki = this->get_parameter("Ki").as_double();
        OUT_RANGE = this->get_parameter("OUT_RANGE").as_int();
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&FollowTheGapNode::lidar_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "existo :D en forma de follow the Gap");
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        double dt = this->get_clock()->now().seconds() - last_time_.seconds();

        // int gap_start, gap_end;

        // std::tie(gap_start, gap_end) = find_biggest_gap(msg->ranges, 180, 0.8);
        float mid_gap = find_biggest_gap(msg->ranges, OUT_RANGE, 0.8);
        mid_gap = mid_gap;
        RCLCPP_INFO(this->get_logger(), "mid_gap: %.2f", mid_gap);

        // int ref = round(msg->ranges.size() / 2);

        int ref = 0;

        double error = (ref - mid_gap);

        double de = error - last_error_;
        last_error_ = error;
        integral_ = (error - last_error_) * dt;
        double theta_d = Kp * error + Kd * de / dt + integral_*Ki;

        geometry_msgs::msg::Twist cmd;
        theta_ant_ = theta_ant_ - theta_d;
        if (abs(theta_ant_) > MAX_TETHA)
        {
            theta_ant_ = theta_ant_ / abs(theta_ant_) * MAX_TETHA;
        }
        if (msg->ranges[0] <= Switch || p_radar<Switch*0.3)
        {        
            
            RCLCPP_INFO(this->get_logger(), "Se activo mi fafa con: %.2f", p_radar);
            cmd.angular.z = theta_ant_;
            cmd.linear.x = VEL_X;
            pub_->publish(cmd);
        }
    }

    float find_biggest_gap(const std::vector<float> &data, int skip_count, float v)
    {
        double sumR=0;
        int size = data.size();
        // Validar que el vector data no esté vacío y que skip_count sea menor o igual a la mitad del tamaño de data
        if (data.empty() || skip_count > size )
        {
            // Retornar un valor por defecto o manejar el error según tus necesidades
            RCLCPP_WARN(this->get_logger(), "No hay datos en el vector");
            return -1.0; // Puedes ajustar este valor por defecto o manejar el error de otra manera
        }

        int gap_start = 0;
        int max_gap_start = 0;
        int max_gap_size = 0;
        int current_gap_size = 0;
        int j = 1;
        
        // Divide el vector en tres grupos: primero, segundo (que se omite) y tercero

        float second_group_start = size / 2 - skip_count / 2;
        float third_group_start = second_group_start + skip_count;
        // RCLCPP_INFO(this->get_logger(), "l:%.2d p: %.2f s: %.2f ", size, second_group_start, third_group_start);

        // Evaluar primero el tercer grupo
        for (int i = third_group_start; i < size; ++i)
        {
            float value = data[i];
            // Tratar valores infinitos como números muy grandes
            if (std::isinf(value))
            {
                value = 30.0;
            }
            sumR+=value;

            if (value > v)
            {
                if (current_gap_size == 0)
                {
                    gap_start = j;
                }
                current_gap_size++;
            }
            else
            {
                if (current_gap_size > max_gap_size)
                {
                    max_gap_size = current_gap_size;
                    max_gap_start = gap_start;
                }
                current_gap_size = 0;
            }
            j++;
        }
        // Evaluar luego el primer grupo
        for (int i = 0; i < second_group_start; ++i)
        {
            float value = data[i];
            // Tratar valores infinitos como números muy grandes
            if (std::isinf(value))
            {
                value = 30.0;
            }
            sumR+=value;
            if (value > v && i != second_group_start - 1)
            {
                if (current_gap_size == 0)
                {
                    gap_start = j;
                }
                current_gap_size++;
            }
            else
            {
                if (current_gap_size > max_gap_size)
                {
                    max_gap_size = current_gap_size;
                    max_gap_start = gap_start;
                }
                current_gap_size = 0;
            }
            j++;
        }


        p_radar=sumR/j;

        current_gap_size = 0;
        double mid_gap = max_gap_start + max_gap_size / 2;
        double gap_base_scalin = (size - skip_count) / 2;
        return (mid_gap - gap_base_scalin);
    }

    rclcpp::Time last_time_ = this->get_clock()->now();
    double theta_ant_;
    double integral_;
    double last_error_ = 0;
    double Switch = 1.0;
    double VEL_X = 0.3;
    double MAX_TETHA = 0.6;
    double Kp = 0.8;   // Adjust as needed
    double Kd = 3.212; // Adjust as needed
    double Ki = 0.5;
    double p_radar=30;
    int OUT_RANGE = 240;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowTheGapNode>());
    rclcpp::shutdown();
    return 0;
}