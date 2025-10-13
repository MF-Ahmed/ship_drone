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

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic, 50,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg){
        geometry_msgs::msg::PoseStamped p;
        p.header = msg->header;
        p.pose   = msg->pose.pose;
        path_.header = msg->header;
        path_.poses.push_back(p);
        if ((int)path_.poses.size() > max_points_) path_.poses.erase(path_.poses.begin());
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
