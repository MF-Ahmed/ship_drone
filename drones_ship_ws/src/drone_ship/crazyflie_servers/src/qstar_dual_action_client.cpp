// qstar_dual_action_client.cpp  (ROS 2 Jazzy)
// Subscribes to:
//   /<ns>/assigned_pose   (PoseStamped)  = q* in WORLD frame (from assignment_node)
//   /<ns>/ekf/odom        (nav_msgs/Odometry) current pose + yaw
//
// Sends actions:
//   /<ns>/navigate_to_hover   (only uses target_z in your server)
//   /<ns>/move_forward        (expects dx,dy in yaw-aligned frame at action start)
//
// Key idea:
//   Given q*=(x*,y*) in WORLD and current (x,y,yaw),
//   compute displacement in yaw-aligned frame that MoveForwardServer uses:
//
//   dx_w = x* - x
//   dy_w = y* - y
//   dx_body =  cos(yaw)*dx_w + sin(yaw)*dy_w
//   dy_body = -sin(yaw)*dx_w + cos(yaw)*dy_w
//
// Then send MoveForward goal: distance_x=dx_body, distance_y=dy_body

#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "crazyflie_servers/action/navigate_to_hover.hpp"
#include "crazyflie_servers/action/move_forward.hpp"

using NavigateToHover = crazyflie_servers::action::NavigateToHover;
using MoveForward     = crazyflie_servers::action::MoveForward;

using namespace std::chrono_literals;

static double yawFromQuat(const geometry_msgs::msg::Quaternion &qmsg)
{
  tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

class QStarDualActionClient : public rclcpp::Node
{
public:
  QStarDualActionClient() : rclcpp::Node("qstar_dual_action_client")
  {
    ns_ = declare_parameter<std::string>("ns", "drone1");

    // Topics
    assigned_pose_topic_ = declare_parameter<std::string>(
      "assigned_pose_topic", "/" + ns_ + "/assigned_pose");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/" + ns_ + "/ekf/odom");

    // Action names
    navigate_action_ = declare_parameter<std::string>(
      "navigate_action", "/" + ns_ + "/navigate_to_hover");
    forward_action_ = declare_parameter<std::string>(
      "forward_action", "/" + ns_ + "/move_forward");

    // Motion params
    hover_sec_ = declare_parameter<int>("hover_sec", 2);
    speed_     = declare_parameter<double>("speed", 0.20);

    // Optional: if you want to command a yaw in hover action (your hover server ignores it)
    yaw_cmd_   = declare_parameter<double>("yaw_cmd", 0.0);

    // If true, do hover first then XY; if false do XY first then hover
    hover_first_ = declare_parameter<bool>("hover_first", true);

    // If true: run once (first q*), then shutdown
    oneshot_ = declare_parameter<bool>("oneshot", true);

    // Subscribers
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(30),
      std::bind(&QStarDualActionClient::odomCb, this, std::placeholders::_1));

    qstar_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      assigned_pose_topic_, rclcpp::QoS(10),
      std::bind(&QStarDualActionClient::qstarCb, this, std::placeholders::_1));

    // Action clients
    nav_client_ = rclcpp_action::create_client<NavigateToHover>(this, navigate_action_);
    fwd_client_ = rclcpp_action::create_client<MoveForward>(this, forward_action_);

    // Small loop to start when ready
    timer_ = create_wall_timer(200ms, std::bind(&QStarDualActionClient::tick, this));

    RCLCPP_INFO(get_logger(),
      "QStarDualActionClient:\n"
      "  ns=%s\n"
      "  assigned_pose_topic=%s\n"
      "  odom_topic=%s\n"
      "  navigate_action=%s\n"
      "  forward_action=%s\n"
      "  speed=%.2f hover_sec=%d hover_first=%s oneshot=%s",
      ns_.c_str(),
      assigned_pose_topic_.c_str(),
      odom_topic_.c_str(),
      navigate_action_.c_str(),
      forward_action_.c_str(),
      speed_, hover_sec_,
      hover_first_ ? "true" : "false",
      oneshot_ ? "true" : "false");
  }

private:
  // Params
  std::string ns_;
  std::string assigned_pose_topic_;
  std::string odom_topic_;
  std::string navigate_action_;
  std::string forward_action_;
  int hover_sec_{2};
  double speed_{0.2};
  double yaw_cmd_{0.0};
  bool hover_first_{true};
  bool oneshot_{true};

  // State (latest)
  std::atomic<bool> have_odom_{false};
  std::atomic<bool> have_qstar_{false};
  std::atomic<bool> running_{false};

  double x_{0}, y_{0}, z_{0}, yaw_{0};
  geometry_msgs::msg::PoseStamped qstar_;

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr qstar_sub_;
  rclcpp_action::Client<NavigateToHover>::SharedPtr nav_client_;
  rclcpp_action::Client<MoveForward>::SharedPtr     fwd_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    z_ = msg->pose.pose.position.z;
    yaw_ = yawFromQuat(msg->pose.pose.orientation);
    have_odom_ = true;
  }

  void qstarCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    qstar_ = *msg;
    have_qstar_ = true;
  }

  void tick()
  {
    if (running_) return;
    if (!have_odom_ || !have_qstar_) return;

    if (!nav_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "NavigateToHover server not ready: %s", navigate_action_.c_str());
      return;
    }
    if (!fwd_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "MoveForward server not ready: %s", forward_action_.c_str());
      return;
    }

    running_ = true;

    // Decide sequence
    if (hover_first_) {
      send_hover_then_move();
    } else {
      send_move_then_hover();
    }
  }

  void send_hover_then_move()
  {
    NavigateToHover::Goal nav_goal;
    nav_goal.target_pose.position.x = 0.0;
    nav_goal.target_pose.position.y = 0.0;
    nav_goal.target_pose.position.z = z_;//qstar_.pose.position.z; // z* in WORLD
    nav_goal.target_pose.orientation.w = 1.0;
    nav_goal.yaw = yaw_cmd_;
    nav_goal.hover_time.sec = hover_sec_;
    nav_goal.hover_time.nanosec = 0;

    auto opts = rclcpp_action::Client<NavigateToHover>::SendGoalOptions();
    opts.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavigateToHover>::WrappedResult &res)
      {
        if (res.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_ERROR(this->get_logger(), "[%s] Hover failed.", ns_.c_str());
          running_ = false;
          if (oneshot_) rclcpp::shutdown();
          return;
        }
        // After hover completes, compute dx/dy using the latest odom (yaw at move start)
        this->send_move_goal_from_qstar();
      };

    RCLCPP_INFO(get_logger(), "[%s] Hover to z*=%.2f (q*)", ns_.c_str(), qstar_.pose.position.z);
    nav_client_->async_send_goal(nav_goal, opts);
  }

  void send_move_then_hover()
  {
    // Move first, then hover at end
    auto after_move = [this](bool ok)
    {
      if (!ok) {
        running_ = false;
        if (oneshot_) rclcpp::shutdown();
        return;
      }

      NavigateToHover::Goal nav_goal;
      nav_goal.target_pose.position.z = z_;//qstar_.pose.position.z;
      nav_goal.target_pose.orientation.w = 1.0;
      nav_goal.yaw = yaw_cmd_;
      nav_goal.hover_time.sec = hover_sec_;
      nav_goal.hover_time.nanosec = 0;

      auto opts = rclcpp_action::Client<NavigateToHover>::SendGoalOptions();
      opts.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<NavigateToHover>::WrappedResult &res)
        {
          const bool ok2 = (res.code == rclcpp_action::ResultCode::SUCCEEDED);
          RCLCPP_INFO(this->get_logger(), "[%s] Final hover done: %s", ns_.c_str(), ok2 ? "OK" : "FAIL");
          running_ = false;
          if (oneshot_) rclcpp::shutdown();
        };

      RCLCPP_INFO(get_logger(), "[%s] Final hover to z*=%.2f", ns_.c_str(), qstar_.pose.position.z);
      nav_client_->async_send_goal(nav_goal, opts);
    };

    send_move_goal_from_qstar(after_move);
  }

  void send_move_goal_from_qstar(std::function<void(bool)> done_cb = {})
  {
    // Current pose (world)
    const double x = x_, y = y_, yaw = yaw_;

    // q* (world)
    const double xg = qstar_.pose.position.x;
    const double yg = qstar_.pose.position.y;

    // World displacement to goal
    const double dx_w = xg - x;
    const double dy_w = yg - y;

    // Convert to yaw-aligned frame used by MoveForwardServer (yaw0 captured at start)
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    const double dx_cmd =  c * dx_w + s * dy_w;
    const double dy_cmd = -s * dx_w + c * dy_w;

    const double d_tot = std::hypot(dx_cmd, dy_cmd);
    if (d_tot < 1e-3) {
      RCLCPP_WARN(get_logger(), "[%s] q* is very close (%.3f m). Skipping move_forward.", ns_.c_str(), d_tot);
      if (done_cb) done_cb(true);
      else {
        running_ = false;
        if (oneshot_) rclcpp::shutdown();
      }
      return;
    }

    MoveForward::Goal fwd_goal;
    fwd_goal.distance_x = dx_cmd;
    fwd_goal.distance_y = dy_cmd;
    fwd_goal.speed      = speed_;

    auto opts = rclcpp_action::Client<MoveForward>::SendGoalOptions();
    opts.feedback_callback =
      [this](auto, const std::shared_ptr<const MoveForward::Feedback> fb)
      {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "[%s] Move FB: rem_x=%.2f rem_y=%.2f rem_total=%.2f",
          ns_.c_str(), fb->remaining_x, fb->remaining_y, fb->remaining_total);
      };

    opts.result_callback =
      [this, done_cb](const rclcpp_action::ClientGoalHandle<MoveForward>::WrappedResult &res)
      {
        const bool ok = (res.code == rclcpp_action::ResultCode::SUCCEEDED);
        RCLCPP_INFO(this->get_logger(), "[%s] Move result: %s", ns_.c_str(), ok ? "OK" : "FAIL");
        if (done_cb) done_cb(ok);
        else {
          running_ = false;
          if (oneshot_) rclcpp::shutdown();
        }
      };

    RCLCPP_INFO(get_logger(),
      "[%s] q* WORLD=(%.3f, %.3f, %.3f), odom=(%.3f, %.3f), yaw=%.2f rad\n"
      "     -> MoveForward goal in yaw-frame: dx=%.3f dy=%.3f (tot=%.3f) speed=%.2f",
      ns_.c_str(),
      xg, yg, qstar_.pose.position.z,
      x, y, yaw,
      dx_cmd, dy_cmd, d_tot, speed_);

    fwd_client_->async_send_goal(fwd_goal, opts);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QStarDualActionClient>());
  rclcpp::shutdown();
  return 0;
}
