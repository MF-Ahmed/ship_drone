#include "rclcpp/rclcpp.hpp"

class ContainerLocalizeNode : public rclcpp::Node {
public:
  ContainerLocalizeNode() : Node("container_localize_node") {
    RCLCPP_INFO(this->get_logger(), "Container localization node initialized.");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ContainerLocalizeNode>());
  rclcpp::shutdown();
  return 0;
}

