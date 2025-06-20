#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <termios.h>
#include <unistd.h>
#include <iostream>

char getch()
{
  struct termios oldt, newt;
  char ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

class TeleopNode : public rclcpp::Node
{
public:
  TeleopNode() : Node("crazyflie_teleop")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/crazyflie/cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "Keyboard Teleop Started: w = up, s = down, h = hover");

    input_thread_ = std::thread(std::bind(&TeleopNode::keyboard_loop, this));
    input_thread_.detach();
  }

private:
  void keyboard_loop()
  {
    while (rclcpp::ok()) {
      char c = getch();
      geometry_msgs::msg::Twist msg;

      if (c == 'w') {
        msg.linear.z = 0.5;
        RCLCPP_INFO(this->get_logger(), "UP");
      } else if (c == 's') {
        msg.linear.z = -0.5;
        RCLCPP_INFO(this->get_logger(), "DOWN");
      } else if (c == 'h') {
        msg.linear.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "HOVER");
      } else {
        continue;
      }

      publisher_->publish(msg);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  std::thread input_thread_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
