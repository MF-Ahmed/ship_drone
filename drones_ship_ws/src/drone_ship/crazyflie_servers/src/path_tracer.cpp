// src/path_tracer.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PathTracer : public rclcpp::Node {
public:
  PathTracer() : Node("path_tracer") {
    std::string odom_topic = declare_parameter<std::string>("odom_topic", "ekf/odom");
    std::string path_topic = declare_parameter<std::string>("path_topic", "trajectory");
    max_points_ = declare_parameter<int>("max_points", 5000);
    if (max_points_ < 1) max_points_ = 5000;

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, rclcpp::QoS(10));

    // initialize path header once; adjust to your TF tree if needed
    path_.header.frame_id = "world";
    path_.poses.reserve(static_cast<size_t>(max_points_));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(50),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {   // <-- ConstSharedPtr
        geometry_msgs::msg::PoseStamped p;
        p.header = msg->header;
        p.pose   = msg->pose.pose;

        // keep header.frame_id consistent
        if (path_.header.frame_id.empty()) {
          path_.header.frame_id = msg->header.frame_id;
        }

        path_.poses.push_back(std::move(p));
        if (static_cast<int>(path_.poses.size()) > max_points_) {
          // drop oldest
          path_.poses.erase(path_.poses.begin());
        }

        // stamp path header (use node clock or pass-through)
        path_.header.stamp = this->get_clock()->now();
        path_pub_->publish(path_);
      });
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Path path_;
  int max_points_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathTracer>());
  rclcpp::shutdown();
  return 0;
}
