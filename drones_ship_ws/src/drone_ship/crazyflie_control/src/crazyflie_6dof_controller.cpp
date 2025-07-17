#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

using namespace std::chrono_literals;

class CrazyflieAutoMission : public rclcpp::Node {
public:
  CrazyflieAutoMission()
  : Node("crazyflie_auto_mission"),
    ascend_altitude_(10.0),
    forward_distance_(30.0),
    hover_time_(5.0),
    vertical_speed_(0.5),
    forward_speed_(0.5),
    stage_(ASCEND),
    initialized_(false)
  {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "ekf/odom", 10,
      std::bind(&CrazyflieAutoMission::odom_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(500ms, std::bind(&CrazyflieAutoMission::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Crazyflie auto mission started.");
  }

private:
  enum Stage { ASCEND, HOVER1, FORWARD, HOVER2, COMPLETE };
  Stage stage_;
  bool initialized_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double ascend_altitude_, forward_distance_, hover_time_;
  double vertical_speed_, forward_speed_;
  rclcpp::Time hover_start_;

  double start_x_, start_y_;
  double current_x_, current_y_, current_z_;
  double roll_, pitch_, yaw_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_z_ = msg->pose.pose.position.z;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);

    if (!initialized_) {
      start_x_ = current_x_;
      start_y_ = current_y_;
      initialized_ = true;
    }
  }

  void control_loop() {
    geometry_msgs::msg::Twist cmd;

    switch (stage_) {
      case ASCEND:
        if (current_z_ < ascend_altitude_ - 0.1) {
          cmd.linear.z = vertical_speed_;
        } else {
          cmd.linear.z = 0.0;
          stage_ = HOVER1;
          hover_start_ = this->get_clock()->now();
          RCLCPP_INFO(this->get_logger(), "Reached altitude %.2f m. Hovering...", ascend_altitude_);
        }
        break;

      case HOVER1:
        cmd.linear.z = 0.0;
        if ((this->get_clock()->now() - hover_start_).seconds() >= hover_time_) {
          stage_ = FORWARD;
          RCLCPP_INFO(this->get_logger(), "Hover complete. Moving forward...");
        }
        break;

      case FORWARD: {
        double dx = current_x_ - start_x_;
        double dy = current_y_ - start_y_;
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance < forward_distance_) {
          cmd.linear.x = forward_speed_;
        } else {
          cmd.linear.x = 0.0;
          stage_ = HOVER2;
          hover_start_ = this->get_clock()->now();
          RCLCPP_INFO(this->get_logger(), "Reached forward target %.2f m. Hovering...", forward_distance_);
        }
        break;
      }

      case HOVER2:
        cmd.linear.x = 0.0;
        if ((this->get_clock()->now() - hover_start_).seconds() >= hover_time_) {
          stage_ = COMPLETE;
          RCLCPP_INFO(this->get_logger(), "Mission complete. Holding position.");
        }
        break;

      case COMPLETE:
        cmd.linear.x = 0.0;
        cmd.linear.z = 0.0;
        break;
    }

    RCLCPP_INFO(this->get_logger(),
      "Stage: %d | Position: [%.2f, %.2f, %.2f] | RPY: [%.2f°, %.2f°, %.2f°] | Cmd: [vx=%.2f, vz=%.2f]",
      stage_, current_x_, current_y_, current_z_,
      roll_ * 180.0 / M_PI, pitch_ * 180.0 / M_PI, yaw_ * 180.0 / M_PI,
      cmd.linear.x, cmd.linear.z);

    cmd_pub_->publish(cmd);
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrazyflieAutoMission>());
  rclcpp::shutdown();
  return 0;
}
