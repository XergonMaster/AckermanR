#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "../lib/serialib.h"
#include <unistd.h>
#include <stdio.h>
#include <iomanip>

class AGVController : public rclcpp::Node
{
public:
  static constexpr const char *SERIAL_PORT = "/dev/ttyS0";
  static constexpr speed_t BAUD_RATE = 115200;
  static constexpr int DATA_BITS = 8;

  double prev_linear_x_;
  double prev_linear_y_;
  double prev_angular_z_;

  explicit AGVController()
      : Node("agv_controller")
  {
    char errorOpening = serial.openDevice(SERIAL_PORT, BAUD_RATE);

    if (errorOpening != 1)
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to connect to %s. Closing node...", SERIAL_PORT);
      rclcpp::shutdown();
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Successful serial connection to %s\n", SERIAL_PORT);
    }

    twist_subs_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_out", rclcpp::QoS(10),
        std::bind(&AGVController::sendCmdVel, this, std::placeholders::_1));

    motor_velocities_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motor_velocities", 10);

    RCLCPP_INFO(this->get_logger(), "agv_controller has started");

    confirmCommunication(); // Confirmar la comunicación con la Raspberry Pi Pico
  }

  void sendCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double linear_x = msg->linear.x;
    double linear_y = msg->linear.y;
    double angular_z = msg->angular.z;

    if (linear_x != prev_linear_x_ || linear_y != prev_linear_y_ || angular_z != prev_angular_z_)
    {

      prev_linear_x_ = linear_x;
      prev_linear_y_ = linear_y;
      prev_angular_z_ = angular_z;

      std::ostringstream oss;
      oss << std::fixed << std::setprecision(1);

      oss << ((linear_x >= 0) ? "+" : "-") << std::setw(1) << std::abs(static_cast<int>(linear_x));
      oss << "." << std::setw(1) << std::abs(static_cast<int>((linear_x - static_cast<int>(linear_x)) * 10));

      oss << ((linear_y >= 0) ? "+" : "-") << std::setw(1) << std::abs(static_cast<int>(linear_y));
      oss << "." << std::setw(1) << std::abs(static_cast<int>((linear_y - static_cast<int>(linear_y)) * 10));

      oss << ((angular_z >= 0) ? "+" : "-") << std::setw(1) << std::abs(static_cast<int>(angular_z));
      oss << "." << std::setw(1) << std::abs(static_cast<int>((angular_z - static_cast<int>(angular_z)) * 10));

      oss << "CTS";

      std::string uart_msg = oss.str();

      sendUART(uart_msg);
    }
    sendUART("............CTS");
  }

  void processOdom()
  {
    const std::string RTS_PREFIX = "RTS";
    char rtsMessage[30];
    serial.readString(rtsMessage, '\n', sizeof(rtsMessage), 3);
    std::string rtsReceived(rtsMessage);

    const std::string ODOM_PREFIX = "ODOM_";
    char odomData[30];
    serial.readString(odomData, '\n', sizeof(odomData), 200);
    std::string odomReceived(odomData);

    if (rtsReceived.substr(0, RTS_PREFIX.size()) == RTS_PREFIX)
    {
      sendUART("............CTS");
    }

    else if (odomReceived.substr(0, ODOM_PREFIX.size()) == ODOM_PREFIX)
    {
      std::string data = odomReceived.substr(ODOM_PREFIX.size());

      std_msgs::msg::Float32MultiArray motor_velocities_msg;

      std::size_t pos = 0;
      std::size_t pos_next = 0;
      float val = 0.0f;

      for (int i = 0; i < 4; i++)
      {
        // Extracción y conversión para cada velocidad de motor
        pos_next = data.find_first_of("+-", pos + 1);
        val = std::stof(data.substr(pos, pos_next - pos));
        motor_velocities_msg.data.push_back(val);
        pos = pos_next;
      }

      motor_velocities_pub_->publish(motor_velocities_msg);
      RCLCPP_INFO(this->get_logger(), "Publishing Motor Velocities");
    }
  }

private:
  char buffer[20];

  void sendUART(const std::string &message)
  {
    serial.writeString(const_cast<char *>(message.c_str()));
    RCLCPP_INFO(this->get_logger(), message.c_str());
  }

  void confirmCommunication()
  {
    const std::string confirmMsg = "RPIPICO_STATUS";
    bool success;

    // Enviar un mensaje y esperar la confirmación "OK"
    sendUART(confirmMsg);
    RCLCPP_INFO(this->get_logger(), confirmMsg.c_str());
    success = waitForResponse("OK", 10000);

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Communication confirmed with Raspberry Pi Pico");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to confirm communication with Raspberry Pi Pico");
      rclcpp::shutdown();
    }
  }

  bool waitForResponse(const std::string &expectedResponse, int timeout_ms)
  {
    const auto startTime = std::chrono::steady_clock::now();
    const auto endTime = startTime + std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < endTime)
    {
      if (serial.readString(buffer, '\n', 20, 200) > 0)
      {
        std::string receivedResponse(buffer);
        if (receivedResponse.find(expectedResponse) != std::string::npos)
        {
          return true;
        }
      }
      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Agregar una pequeña pausa para evitar un bucle de espera activa
    }
    RCLCPP_INFO(this->get_logger(), "It was not possible to init UART. RPiPico is not answering\n Retrying...");
    return false;
  }

  serialib serial;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subs_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_velocities_pub_;
  rclcpp::TimerBase::SharedPtr serial_read_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto agv_controller = std::make_shared<AGVController>();

  auto timer = agv_controller->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&AGVController::processOdom, agv_controller));

  rclcpp::spin(agv_controller);
  rclcpp::shutdown();

  return 0;
}