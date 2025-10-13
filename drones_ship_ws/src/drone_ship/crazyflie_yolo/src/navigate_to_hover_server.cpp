// navigate_to_hover_server.cpp — ROS 2 Jazzy
// Action: crazyflie_yolo/NavigateToHover
// Publishes cmd_vel.linear.z to reach a target altitude from /<ns>/ekf/odom,
// then hovers for goal.hover_time seconds. Supports cancel.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>

#include "crazyflie_yolo/action/navigate_to_hover.hpp"

using NavigateToHover = crazyflie_yolo::action::NavigateToHover;
using GoalHandleHover = rclcpp_action::ServerGoalHandle<NavigateToHover>;
using namespace std::chrono_literals;

class NavigateToHoverServer : public rclcpp::Node {
public:
  NavigateToHoverServer()
  : Node("navigate_to_hover_server")
  {
    // Parameters (can be set per-namespace)
    odom_topic_    = declare_parameter<std::string>("odom_topic", "ekf/odom");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");

    // Controller params
    kp_ = declare_parameter<double>("kp", 0.6);
    ki_ = declare_parameter<double>("ki", 0.08);
    kd_ = declare_parameter<double>("kd", 0.0);
    vz_max_ = declare_parameter<double>("vz_max", 0.5);   // m/s
    vz_min_ = declare_parameter<double>("vz_min", -0.5);  // m/s
    z_tol_  = declare_parameter<double>("z_tolerance", 0.05); // m
    i_limit_ = declare_parameter<double>("i_limit", 1.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg){
        current_z_ = msg->pose.pose.position.z;
      });

    server_ = rclcpp_action::create_server<NavigateToHover>(
      this, "navigate_to_hover",
      std::bind(&NavigateToHoverServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NavigateToHoverServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&NavigateToHoverServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "Action server up: navigate_to_hover (odom='%s', cmd='%s')",
      odom_topic_.c_str(), cmd_vel_topic_.c_str());
  }

private:
  // ROS I/O
  rclcpp_action::Server<NavigateToHover>::SharedPtr server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Params
  std::string odom_topic_, cmd_vel_topic_;
  double kp_, ki_, kd_, vz_max_, vz_min_, z_tol_, i_limit_;

  // State
  std::atomic<double> current_z_{0.0};

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &,
      std::shared_ptr<const NavigateToHover::Goal> goal)
  {
    RCLCPP_INFO(get_logger(),
      "Goal: (%.2f, %.2f, %.2f), yaw=%.2f, hover=%.1fs",
      goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z,
      goal->yaw, goal->hover_time.sec + goal->hover_time.nanosec*1e-9);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleHover>)
  {
    RCLCPP_INFO(get_logger(), "Cancel request");
    publish_zero();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleHover> gh)
  {
    std::thread{std::bind(&NavigateToHoverServer::execute, this, gh)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleHover> gh)
  {
    rclcpp::Rate rate(50.0); // 50 Hz
    auto feedback = std::make_shared<NavigateToHover::Feedback>();
    auto result   = std::make_shared<NavigateToHover::Result>();
    const auto goal = gh->get_goal();

    const double z_goal = goal->target_pose.position.z;
    const double hover_s = goal->hover_time.sec + goal->hover_time.nanosec*1e-9;

    // PID state
    double integ = 0.0;
    double prev_err = 0.0;

    enum Phase { ASCEND, HOVER } phase = ASCEND;
    rclcpp::Time hover_start;

    while (rclcpp::ok()) {
      if (gh->is_canceling()) {
        publish_zero();
        result->success = false; result->message = "Canceled";
        gh->canceled(result);
        RCLCPP_INFO(get_logger(), "Goal canceled");
        return;
      }

      double z = current_z_.load();
      double err = z_goal - z;

      // Feedback (distance_to_target ~ |z error| for this vertical controller)
      feedback->distance_to_target = std::fabs(err);
      feedback->remaining_hover_s  = (phase == HOVER) ? std::max(0.0, hover_s - (now() - hover_start).seconds()) : hover_s;
      gh->publish_feedback(feedback);

      geometry_msgs::msg::Twist cmd;

      if (phase == ASCEND) {
        // PID with simple anti-windup
        double dt = 1.0/50.0;
        double deriv = (err - prev_err) / dt;
        double integ_cand = std::clamp(integ + err * dt, -i_limit_, i_limit_);
        double vz_unsat = kp_*err + ki_*integ_cand + kd_*deriv;
        double vz = std::clamp(vz_unsat, vz_min_, vz_max_);

        // anti-windup: only accept integral if not saturating against error
        bool sat_up   = (vz_unsat > vz) && (err > 0.0);
        bool sat_down = (vz_unsat < vz) && (err < 0.0);
        if (!(sat_up || sat_down)) integ = integ_cand;

        prev_err = err;
        cmd.linear.z = vz;

        // Transition to hover once within tolerance
        if (std::fabs(err) <= z_tol_) {
          hover_start = now();
          phase = HOVER;
          integ = 0.0; prev_err = 0.0; // reset
        }
      } else { // HOVER
        // Hold altitude with tighter limits
        double dt = 1.0/50.0;
        double deriv = (err - prev_err) / dt;
        double integ_cand = std::clamp(integ + err * dt, -i_limit_, i_limit_);
        double vz_unsat = kp_*err + ki_*integ_cand + kd_*deriv;
        double vz = std::clamp(vz_unsat, -0.2, 0.2);
        bool sat_up   = (vz_unsat > vz) && (err > 0.0);
        bool sat_down = (vz_unsat < vz) && (err < 0.0);
        if (!(sat_up || sat_down)) integ = integ_cand;
        prev_err = err;
        cmd.linear.z = vz;

        if ((now() - hover_start).seconds() >= hover_s) {
          publish_zero();
          result->success = true;
          result->message = "Arrived and hovered";
          result->final_pose = goal->target_pose;
          gh->succeed(result);
          RCLCPP_INFO(get_logger(), "Goal succeeded");
          return;
        }
      }

      cmd_pub_->publish(cmd);
      rate.sleep();
    }

    publish_zero();
  }

  void publish_zero()
  {
    geometry_msgs::msg::Twist zero;
    cmd_pub_->publish(zero);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigateToHoverServer>());
  rclcpp::shutdown();
  return 0;
}
