#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using namespace std::chrono_literals;

class CrazyflieForwardNode : public rclcpp::Node {
public:
  CrazyflieForwardNode()
  : Node("crazyflie_forward_node"),
    target_z_(8.0),
    target_distance_(18.0),
    stage_(ASCEND),
    pid_z_(0.4, 0.01, 0.1),
    pid_forward_(0.15, 0.0, 0.03),  // slightly reduced gains for smoother motion
    pid_yaw_(1.0, 0.0, 0.1),
    forward_speed_limit_(0.25),     // higher speed limit for forward
    forward_ramp_rate_(0.02),       // gradual ramp up (m/s per iteration)
    smoothed_cmd_x_(0.0),
    forward_initialized_(false)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "ekf/odom", 10,
      std::bind(&CrazyflieForwardNode::odom_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(100ms, std::bind(&CrazyflieForwardNode::control_loop, this));
    RCLCPP_INFO(this->get_logger(), "Crazyflie ascend + smooth forward node started.");
  }

private:
  enum Stage { ASCEND, HOVER, FORWARD, STOP };
  Stage stage_;

  struct PID {
    double kp, ki, kd;
    double integral = 0.0;
    double previous_error = 0.0;

    PID(double p, double i, double d) : kp(p), ki(i), kd(d) {}

    double compute(double error, double dt) {
      integral += error * dt;
      double derivative = (error - previous_error) / dt;
      previous_error = error;
      return kp * error + ki * integral + kd * derivative;
    }
  };

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_z_ = msg->pose.pose.position.z;
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_orientation_ = msg->pose.pose.orientation;

    if (!forward_initialized_) {
      start_x_ = current_x_;
      start_y_ = current_y_;
      hover_start_time_ = this->get_clock()->now();
      initial_yaw_ = tf2::getYaw(current_orientation_);
      forward_initialized_ = true;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->get_clock()->now();
    pose.header.frame_id = msg->header.frame_id;
    pose.pose = msg->pose.pose;
    path_.poses.push_back(pose);
    path_.header.stamp = this->get_clock()->now();
    path_.header.frame_id = msg->header.frame_id;

    if (path_.poses.size() > 5000) {
      path_.poses.erase(path_.poses.begin());
    }
    path_pub_->publish(path_);
  }

  void control_loop() {
    geometry_msgs::msg::Twist cmd;
    double dt = 0.1;

    switch (stage_) {
      case ASCEND: {
        double error_z = target_z_ - current_z_;
        if (std::abs(error_z) < 0.05) {
          hover_start_time_ = this->get_clock()->now();
          stage_ = HOVER;
          RCLCPP_INFO(this->get_logger(), "Reached target altitude. Hovering briefly...");
          break;
        }
        double vz = pid_z_.compute(error_z, dt);
        cmd.linear.z = std::clamp(vz, -0.3, 0.3);
        break;
      }

      case HOVER: {
        cmd.linear.z = 0.0;
        if ((this->get_clock()->now() - hover_start_time_).seconds() > 2.0) {
          stage_ = FORWARD;
          smoothed_cmd_x_ = 0.0;  // reset smoothed speed
          RCLCPP_INFO(this->get_logger(), "Hover done. Moving forward...");
        }
        break;
      }
           
      
      case FORWARD: {
        double dx = current_x_ - start_x_;
        double dy = current_y_ - start_y_;
        double forward_progress = dx * std::cos(initial_yaw_) + dy * std::sin(initial_yaw_);
        double error_forward = target_distance_ - forward_progress;

        if (error_forward < 0.05) {
          stage_ = STOP;
          RCLCPP_INFO(this->get_logger(), "Reached 30m forward. Stopping.");
          break;
        }

        double current_yaw = tf2::getYaw(current_orientation_);
        double yaw_error = initial_yaw_ - current_yaw;
        if (yaw_error > M_PI) yaw_error -= 2 * M_PI;
        if (yaw_error < -M_PI) yaw_error += 2 * M_PI;

        double wz = pid_yaw_.compute(yaw_error, dt);
        wz = std::clamp(wz, -0.3, 0.3);

        // Smooth forward velocity ramp-up
        double desired_vx = 0.0;
        if (std::abs(yaw_error) < 0.1) {
          desired_vx = pid_forward_.compute(error_forward, dt);
          desired_vx = std::clamp(desired_vx, -forward_speed_limit_, forward_speed_limit_);
        } else {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Aligning yaw before moving forward...");
        }

        // Smooth velocity using ramping
        if (smoothed_cmd_x_ < desired_vx) {
          smoothed_cmd_x_ += forward_ramp_rate_;
          if (smoothed_cmd_x_ > desired_vx) smoothed_cmd_x_ = desired_vx;
        } else {
          smoothed_cmd_x_ -= forward_ramp_rate_;
          if (smoothed_cmd_x_ < desired_vx) smoothed_cmd_x_ = desired_vx;
        }

        // Tilt compensation
        double pitch_angle = std::atan2(smoothed_cmd_x_, 9.81); // approximate pitch
        double compensation_factor = 1.0 / std::cos(pitch_angle);

        double error_z = target_z_ - current_z_;
        double vz = pid_z_.compute(error_z, dt);
        vz = std::clamp(vz * compensation_factor, -0.3, 0.3);

        cmd.linear.x = smoothed_cmd_x_ * std::cos(initial_yaw_);
        cmd.linear.y = smoothed_cmd_x_ * std::sin(initial_yaw_);
        cmd.linear.z = vz;
        cmd.angular.z = wz;
        break;
      }
      
      case STOP:
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.z = 0.0;
        break;
    }

    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    publisher_->publish(cmd);

    double dx = current_x_ - start_x_;
    double dy = current_y_ - start_y_;
    double projected_dist = dx * std::cos(initial_yaw_) + dy * std::sin(initial_yaw_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Stage: %d | Z: %.2f | X: %.2f | Y: %.2f | ΔFwd: %.2f / %.2f",
      stage_, current_z_, current_x_, current_y_, projected_dist, target_distance_);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  PID pid_z_;
  PID pid_forward_;
  PID pid_yaw_;

  double target_z_;
  double target_distance_;
  double forward_speed_limit_;
  double forward_ramp_rate_;
  double current_z_;
  double current_x_;
  double current_y_;
  double start_x_;
  double start_y_;
  double smoothed_cmd_x_;
  bool forward_initialized_;
  rclcpp::Time hover_start_time_;
  double initial_yaw_;
  geometry_msgs::msg::Quaternion current_orientation_;

  nav_msgs::msg::Path path_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrazyflieForwardNode>());
  rclcpp::shutdown();
  return 0;
}
