#include "rclcpp/rclcpp.hpp"
#include "crazyflie_ekf/crazyflie_ekf.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("crazyflie_ekf_node");

  // Create EKF object (subscribes, publishes, etc.)
  auto ekf = std::make_shared<crazyflie_ekf::CrazyflieEKF>(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
