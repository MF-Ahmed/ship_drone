// move_forward_server.cpp — ROS 2 Jazzy
// Action: crazyflie_servers/MoveForward (extended)
// Goal fields: distance_x, distance_y, speed
// Features: XY motion in yaw-aligned frame, world/body cmd frames, yaw hold with gating,
//           smooth ramp, proactive braking, slow zone taper, time-in-tolerance hysteresis.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <algorithm>
#include <atomic>
#include <string>

#include "crazyflie_servers/action/move_forward.hpp"

using MoveForward   = crazyflie_servers::action::MoveForward;
using GoalHandleMF  = rclcpp_action::ServerGoalHandle<MoveForward>;

static inline double wrapToPi(double a) {
  while (a >  M_PI) a -= 2.0*M_PI;
  while (a < -M_PI) a += 2.0*M_PI;
  return a;
}

class MoveForwardServer : public rclcpp::Node {
public:
  MoveForwardServer() : rclcpp::Node("move_forward_server") {
    // I/O topics
    odom_topic_    = declare_parameter<std::string>("odom_topic",    "ekf/odom");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");

    // Motion / control params
    max_speed_      = declare_parameter<double>("max_speed",        0.6);     // clamp for goal.speed
    speed_limit_    = declare_parameter<double>("speed_limit",      0.6);     // hard cap
    ramp_rate_      = declare_parameter<double>("forward_ramp_rate",0.02);    // m/s per tick @50Hz ~1 m/s^2
    stop_tolerance_ = declare_parameter<double>("stop_tolerance",   0.15);    // m (radius)
    brake_margin_   = declare_parameter<double>("brake_margin",     6.0);     // >1 → brake earlier
    slow_radius_    = declare_parameter<double>("slow_radius",      1.0);     // taper zone (m)

    // Yaw hold
    cmd_frame_      = declare_parameter<std::string>("cmd_frame",   "body");  // "world" | "body"
    hold_yaw_       = declare_parameter<bool>("hold_yaw",           true);
    yaw_kp_         = declare_parameter<double>("yaw_kp",           0.6);
    yaw_rate_max_   = declare_parameter<double>("yaw_rate_max",     0.3);     // rad/s
    yaw_gate_rad_   = declare_parameter<double>("yaw_gate_radius",  0.6);     // m
    yaw_min_speed_  = declare_parameter<double>("yaw_min_speed",    0.05);    // m/s



    // New params
    hold_altitude_ = declare_parameter<bool>("hold_altitude", true);          
    z_kp_          = declare_parameter<double>("z_hold_kp", 1.0);             
    z_vmax_        = declare_parameter<double>("z_hold_vmax", 0.5);           
    z_deadband_    = declare_parameter<double>("z_hold_deadband", 0.02);     

    // Hysteresis at goal
    stop_hold_time_ = declare_parameter<double>("stop_hold_time",   0.2);     // s

    // ROS I/O
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 50, std::bind(&MoveForwardServer::onOdom, this, std::placeholders::_1));

    server_ = rclcpp_action::create_server<MoveForward>(
      this, "move_forward",
      std::bind(&MoveForwardServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveForwardServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveForwardServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
        "MoveForward (XY) server on '%s'  odom='%s' cmd='%s' frame=%s hold_yaw=%s hold_alt=%s", 
        "move_forward", odom_topic_.c_str(), cmd_vel_topic_.c_str(),
        cmd_frame_.c_str(), hold_yaw_ ? "true" : "false", hold_altitude_ ? "true" : "false");   
  }

private:
  // ROS
  rclcpp_action::Server<MoveForward>::SharedPtr server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Params
  std::string odom_topic_, cmd_vel_topic_, cmd_frame_;
  double max_speed_{0.6}, speed_limit_{0.6}, ramp_rate_{0.02}, stop_tolerance_{0.15}, brake_margin_{6.0}, slow_radius_{1.0};
  bool   hold_yaw_{true};
  double yaw_kp_{0.6}, yaw_rate_max_{0.3}, yaw_gate_rad_{0.6}, yaw_min_speed_{0.05};
  double stop_hold_time_{0.2};


  bool   hold_altitude_{true};                                             
  double z_kp_{1.0}, z_vmax_{0.5}, z_deadband_{0.02};                     
  double z_ref_{0.0};                                                     

  // Odom state
  std::atomic<double> x_{0.0}, y_{0.0}, z_{0.0}, yaw_{0.0};               

  // Exec state
  double smoothed_v_{0.0};
  bool   in_tol_{false};
  rclcpp::Time tol_enter_time_;

  // Odom callback
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    z_ = msg->pose.pose.position.z;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_ = yaw;
  }

  // Action handlers
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
      std::shared_ptr<const MoveForward::Goal> goal) {
    const double d = std::hypot(goal->distance_x, goal->distance_y);
    if (d <= 1e-6) {
      RCLCPP_WARN(get_logger(), "Rejecting MoveForward goal: near-zero displacement (%.3f m)", d);
      return rclcpp_action::GoalResponse::REJECT;
    }
    const double spd = std::clamp(goal->speed, 0.05, std::min(max_speed_, speed_limit_));
    RCLCPP_INFO(get_logger(), "MoveForward goal: dx=%.2f dy=%.2f spd=%.2f (clamped)",
                goal->distance_x, goal->distance_y, spd);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMF>) {
    RCLCPP_INFO(get_logger(), "MoveForward: cancel requested");
    stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMF> gh) {
    std::thread(&MoveForwardServer::execute, this, gh).detach();
  }

  void execute(const std::shared_ptr<GoalHandleMF> gh) {
    rclcpp::Rate rate(50.0); // 50 Hz
    auto feedback = std::make_shared<MoveForward::Feedback>();
    auto result   = std::make_shared<MoveForward::Result>();

    const auto goal = gh->get_goal();
    const double v_cruise = std::clamp(goal->speed, 0.05, std::min(max_speed_, speed_limit_));

    // Capture start pose + yaw (defines yaw-aligned frame)
    const double x0   = x_.load();
    const double y0   = y_.load();
    z_ref_            = z_.load();  
    const double yaw0 = yaw_.load();

    const double target_x = goal->distance_x;
    const double target_y = goal->distance_y;

    smoothed_v_ = 0.0;
    in_tol_ = false;

    while (rclcpp::ok()) {
      if (gh->is_canceling()) {
        stop();
        result->success = false; result->message = "canceled";
        gh->canceled(result); return;
      }

      // Displacement in world since start
      const double dx_w = x_.load() - x0;
      const double dy_w = y_.load() - y0;

      // Rotate into yaw-aligned frame
      const double c0 = std::cos(yaw0), s0 = std::sin(yaw0);
      const double prog_x =  dx_w * c0 + dy_w * s0;
      const double prog_y = -dx_w * s0 + dy_w * c0;

      double rem_x = target_x - prog_x;
      double rem_y = target_y - prog_y;
      const double rem_tot = std::hypot(rem_x, rem_y);

      // Feedback
      feedback->remaining_x = rem_x;
      feedback->remaining_y = rem_y;
      feedback->remaining_total = rem_tot;
      gh->publish_feedback(feedback);

      // Success check with hysteresis
      if (rem_tot <= stop_tolerance_) {
        if (!in_tol_) { in_tol_ = true; tol_enter_time_ = now(); }
        if ((now() - tol_enter_time_).seconds() >= stop_hold_time_) {
          stop();
          result->success = true; result->message = "completed";
          gh->succeed(result); return;
        }
      } else {
        in_tol_ = false; // left the tolerance band
      }

      // Desired speed (cruise, physics cap, slow zone taper)
      double v_des = std::min(v_cruise, speed_limit_);

      const double dt = 1.0 / 50.0;
      const double a  = std::max(1e-3, ramp_rate_ / dt); // m/s^2 from ramp_rate per tick
      const double v_cap = std::sqrt(std::max(0.0, 2.0 * a * rem_tot / std::max(1.0, brake_margin_)));
      v_des = std::min(v_des, v_cap);          // physically stoppable this cycle

      double v_dist = v_cruise;
      if (rem_tot < slow_radius_) {
        v_dist = v_cruise * (rem_tot / slow_radius_); // linear taper in close
      }
      v_des = std::min(v_des, v_dist);

      // Also avoid single-step overshoot
      v_des = std::min(v_des, rem_tot / dt);

      if (rem_tot < 0.5) {
        v_des = std::min(v_des, 0.6 * rem_tot);   // e.g., 0.3 m left → ≤0.18 m/s
      }
      if (rem_tot < 0.2) {
        v_des = std::min(v_des, 0.3 * rem_tot);   // e.g., 0.1 m left → ≤0.03 m/s
      }      




      // Ramp smoothed_v_ toward v_des
      if (smoothed_v_ < v_des) smoothed_v_ = std::min(smoothed_v_ + ramp_rate_, v_des);
      else                     smoothed_v_ = std::max(smoothed_v_ - ramp_rate_, v_des);

      // Direction in yaw-aligned frame
      double dir_x = 0.0, dir_y = 0.0;
      if (rem_tot > 1e-6) { dir_x = rem_x / rem_tot; dir_y = rem_y / rem_tot; }

      // Velocity in yaw-aligned frame
      const double vx_yaw = smoothed_v_ * dir_x;
      const double vy_yaw = smoothed_v_ * dir_y;

      // Convert desired vel to WORLD
      const double vx_world =  vx_yaw * c0 + vy_yaw * -s0;
      const double vy_world =  vx_yaw * s0 + vy_yaw *  c0;

      // Command frame mapping
      double vx_cmd = vx_world, vy_cmd = vy_world;
      const double yaw_now = yaw_.load();
      const double yaw_err = wrapToPi(yaw0 - yaw_now);

      if (cmd_frame_ == "body") {
        const double cb = std::cos(-yaw_now), sb = std::sin(-yaw_now);
        const double bx =  cb * vx_world - sb * vy_world;
        const double by =  sb * vx_world + cb * vy_world;
        vx_cmd = bx; vy_cmd = by;
      }

      // Yaw hold with gating near goal/low speed (prevents spinning in place)
      double wz_cmd = 0.0;
      const bool gate_yaw = (rem_tot < yaw_gate_rad_) || (smoothed_v_ < yaw_min_speed_);
      if (hold_yaw_ && !gate_yaw) {
        wz_cmd = std::clamp(yaw_kp_ * yaw_err, -yaw_rate_max_, yaw_rate_max_);
      }




      // Altitude hold                                                       
      const double z_now  = z_.load();                                      // 
      const double z_err  = z_ref_ - z_now;                                 // 
      double vz_cmd = 0.0;                                                  // 
      if (hold_altitude_) {                                                 // 
        if (std::abs(z_err) > z_deadband_) {                                // 
          vz_cmd = std::clamp(z_kp_ * z_err, -z_vmax_, z_vmax_);            // 
        }                                                                   // 
      }                      

      // Publish
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x  = vx_cmd;
      cmd.linear.y  = vy_cmd;
      cmd.linear.z  = vz_cmd;
      cmd.angular.z = wz_cmd;
      cmd_pub_->publish(cmd);

      // Diagnostics (1 Hz)
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "XY: prog=(%.2f,%.2f) rem=(%.2f,%.2f | %.2f) v=%.2f yaw_err=%.2fdeg wz=%.2f",
        prog_x, prog_y, rem_x, rem_y, rem_tot, smoothed_v_, yaw_err*180.0/M_PI, wz_cmd);

      rate.sleep();
    }

    stop(); // node shutting down
  }

  void stop() {
    geometry_msgs::msg::Twist zero;
    cmd_pub_->publish(zero);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveForwardServer>());
  rclcpp::shutdown();
  return 0;
}
