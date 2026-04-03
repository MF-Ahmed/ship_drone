#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include "builtin_interfaces/msg/duration.hpp"

#include "crazyflie_yolo/msg/assign_target.hpp"
#include "crazyflie_yolo/msg/tracked_obstacle.hpp"
#include "crazyflie_yolo/msg/tracked_obstacle_array.hpp"

// ---- Actions (from this package) ----
#include "crazyflie_servers/action/navigate_to_hover.hpp"
#include "crazyflie_servers/action/move_forward.hpp"

using namespace std::chrono_literals;
namespace msgs = crazyflie_yolo::msg;
namespace act  = crazyflie_servers::action;

class SurveillancePatrolClient : public rclcpp::Node
{
public:
  SurveillancePatrolClient()
  : Node("surveillance_patrol_client")
  {
    // ---------------- Params ----------------
    drone_ns_   = this->declare_parameter<std::string>("drone_ns", "/drone1");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", drone_ns_ + "/ekf/odom");

    // Surveillance pattern
    patrol_hover_h_   = this->declare_parameter<double>("surveillance_hover_h", 10.0);  // +Z meters
    patrol_forward_m_ = this->declare_parameter<double>("surveillance_forward", 10.0);  // forward meters
    forward_speed_    = this->declare_parameter<double>("forward_speed", 1.5);          // m/s
    hover_dwell_s_    = this->declare_parameter<double>("hover_dwell_s", 2.0);          // s dwell after hover

    // Tracking thresholds/timeouts
    tau_logdet_       = this->declare_parameter<double>("tau_logdet", 0.02);
    track_timeout_s_  = this->declare_parameter<double>("track_timeout_s", 12.0);

    // Topics
    assign_topic_ = this->declare_parameter<std::string>("assign_topic", drone_ns_ + "/assign_target");
    fused_topic_  = this->declare_parameter<std::string>("fused_topic", "fused_tracked_obstacles_array");

    // ---------------- Subscriptions ----------------
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { last_odom_ = *msg; have_odom_ = true; });

    assign_sub_ = this->create_subscription<msgs::AssignTarget>(
      assign_topic_, 10,
      std::bind(&SurveillancePatrolClient::assignCallback, this, std::placeholders::_1));

    fused_sub_ = this->create_subscription<msgs::TrackedObstacleArray>(
      fused_topic_, 10,
      std::bind(&SurveillancePatrolClient::fusedCallback, this, std::placeholders::_1));

    // ---------------- Action clients ----------------
    nav_client_ = rclcpp_action::create_client<act::NavigateToHover>(
      this, drone_ns_ + "/navigate_to_hover");
    fwd_client_ = rclcpp_action::create_client<act::MoveForward>(
      this, drone_ns_ + "/move_forward");

    waitForActionServer(nav_client_, "navigate_to_hover");
    waitForActionServer(fwd_client_, "move_forward");

    // ---------------- Timers (main loop) ----------------
    loop_timer_ = this->create_wall_timer(500ms, std::bind(&SurveillancePatrolClient::loop, this));

    RCLCPP_INFO(get_logger(),
      "surveillance_patrol_client up for ns=%s | odom=%s | assign=%s | fused=%s",
      drone_ns_.c_str(), odom_topic_.c_str(), assign_topic_.c_str(), fused_topic_.c_str());
  }

private:
  // ============================ State ============================
  enum class Mode { SURVEILLANCE, TRACKING };
  Mode mode_ = Mode::SURVEILLANCE;

  // Params
  std::string drone_ns_, odom_topic_, assign_topic_, fused_topic_;
  double patrol_hover_h_{10.0}, patrol_forward_m_{10.0}, forward_speed_{1.5}, hover_dwell_s_{2.0};
  double tau_logdet_{0.02}, track_timeout_s_{12.0};

  // Action clients
  rclcpp_action::Client<act::NavigateToHover>::SharedPtr nav_client_;
  rclcpp_action::Client<act::MoveForward>::SharedPtr     fwd_client_;

  // Subscriptions & last data
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<msgs::AssignTarget>::SharedPtr      assign_sub_;
  rclcpp::Subscription<msgs::TrackedObstacleArray>::SharedPtr fused_sub_;

  nav_msgs::msg::Odometry last_odom_;
  bool have_odom_{false};

  // Assignment / tracking
  int current_target_id_{-1};
  geometry_msgs::msg::PoseStamped current_hover_pose_;  // from AssignTarget
  rclcpp::Time last_seen_target_{0,0, RCL_ROS_TIME};    // updated when fused obs for target_id is seen

  // Patrol bookkeeping
  bool patrol_step_hover_done_{false};
  bool patrol_step_forward_done_{false};
  rclcpp::Time patrol_hover_sent_{0,0, RCL_ROS_TIME};

  // Timer
  rclcpp::TimerBase::SharedPtr loop_timer_;

  // ============================ Callbacks ============================

  void assignCallback(const msgs::AssignTarget::SharedPtr msg)
  {
    if (msg->target_id < 0) {
      if (current_target_id_ >= 0) {
        RCLCPP_INFO(get_logger(), "[%s] assignment cleared -> SURVEILLANCE", drone_ns_.c_str());
      }
      current_target_id_ = -1;
      mode_ = Mode::SURVEILLANCE;
      return;
    }

    current_target_id_ = msg->target_id;
    current_hover_pose_.header.frame_id = "world";
    current_hover_pose_.header.stamp = now();
    current_hover_pose_.pose = msg->hover_pose_world;

    mode_ = Mode::TRACKING;
    patrol_step_hover_done_ = patrol_step_forward_done_ = false;
    RCLCPP_INFO(get_logger(), "[%s] got assignment target_id=%d -> TRACKING",
                drone_ns_.c_str(), current_target_id_);
  }

  void fusedCallback(const msgs::TrackedObstacleArray::SharedPtr arr)
  {
    if (current_target_id_ < 0) return;

    for (const auto & ob : arr->obstacles) {
      if (ob.id != current_target_id_) continue;

      last_seen_target_ = now();

      // Compute log(det(P))
      double P[3][3];
      P[0][0] = ob.covariance[0]; P[0][1] = ob.covariance[1]; P[0][2] = ob.covariance[2];
      P[1][0] = ob.covariance[3]; P[1][1] = ob.covariance[4]; P[1][2] = ob.covariance[5];
      P[2][0] = ob.covariance[6]; P[2][1] = ob.covariance[7]; P[2][2] = ob.covariance[8];

      const double det =
        P[0][0]*(P[1][1]*P[2][2] - P[1][2]*P[2][1]) -
        P[0][1]*(P[1][0]*P[2][2] - P[1][2]*P[2][0]) +
        P[0][2]*(P[1][0]*P[2][1] - P[1][1]*P[2][0]);

      const double det_clamped = std::max(det, 1e-12);
      const double logdet = std::log(det_clamped);

      if (logdet <= tau_logdet_) {
        RCLCPP_INFO(get_logger(),
          "[%s] target %d reached logdet=%.4g <= %.4g -> SURVEILLANCE",
          drone_ns_.c_str(), current_target_id_, logdet, tau_logdet_);
        current_target_id_ = -1;
        mode_ = Mode::SURVEILLANCE;
      }
      break;
    }
  }

  // ============================ Main loop ============================

  void loop()
  {
    if (!have_odom_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
        "[%s] waiting for odom...", drone_ns_.c_str());
      return;
    }

    if (mode_ == Mode::SURVEILLANCE) {
      runSurveillance();
    } else {
      runTracking();
    }
  }

  // ---- Surveillance: hover up, then move forward ----
  void runSurveillance()
  {
    if (current_target_id_ >= 0) return; // assignment arrived

    const auto nowt = now();

    if (!patrol_step_hover_done_) {
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id = last_odom_.header.frame_id.empty() ? "world" : last_odom_.header.frame_id;
      goal.header.stamp = nowt;

      goal.pose.position.x = last_odom_.pose.pose.position.x;
      goal.pose.position.y = last_odom_.pose.pose.position.y;
      goal.pose.position.z = last_odom_.pose.pose.position.z + patrol_hover_h_;
      goal.pose.orientation.w = 1.0;

      bool ok = sendNavigateToHover(goal, hover_dwell_s_);
      if (ok) {
        patrol_step_hover_done_ = true;
        patrol_hover_sent_ = nowt;
        RCLCPP_INFO(get_logger(), "[%s] Surveillance: hovered up by %.1fm",
                    drone_ns_.c_str(), patrol_hover_h_);
      }
      return;
    }

    if ((nowt - patrol_hover_sent_).seconds() < hover_dwell_s_) return;

    if (!patrol_step_forward_done_) {
      // Forward along X (server interprets frame; adapt if your server uses world/body)
      bool ok = sendMoveForward(patrol_forward_m_, 0.0, forward_speed_);
      if (ok) {
        patrol_step_forward_done_ = true;
        RCLCPP_INFO(get_logger(), "[%s] Surveillance: moved forward %.1fm",
                    drone_ns_.c_str(), patrol_forward_m_);
      }
      return;
    }

    patrol_step_hover_done_ = patrol_step_forward_done_ = false; // next cycle
  }

  // ---- Tracking: keep hovering at assigned pose; exit when uncertainty small or timeout ----
  void runTracking()
  {
    const auto nowt = now();

    if (current_target_id_ >= 0 && last_seen_target_.nanoseconds() > 0) {
      if ((nowt - last_seen_target_).seconds() > track_timeout_s_) {
        RCLCPP_WARN(get_logger(), "[%s] target %d timed out (>%0.1fs) -> SURVEILLANCE",
                    drone_ns_.c_str(), current_target_id_, track_timeout_s_);
        current_target_id_ = -1;
        mode_ = Mode::SURVEILLANCE;
        return;
      }
    }

    if (current_target_id_ < 0) {
      mode_ = Mode::SURVEILLANCE;
      return;
    }

    bool ok = sendNavigateToHover(current_hover_pose_, hover_dwell_s_);
    if (!ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "[%s] NavigateToHover retrying...", drone_ns_.c_str());
    }
  }

  // ============================ Actions ============================

  template<typename ActionT>
  void waitForActionServer(const std::shared_ptr<rclcpp_action::Client<ActionT>> & client,
                           const std::string & name)
  {
    if (!client) return;
    RCLCPP_INFO(get_logger(), "Waiting for %s action server...", name.c_str());
    while (rclcpp::ok() && !client->wait_for_action_server(1s)) {
      RCLCPP_INFO(get_logger(), "  still waiting for %s ...", name.c_str());
    }
    RCLCPP_INFO(get_logger(), "Connected to %s.", name.c_str());
  }

  static builtin_interfaces::msg::Duration toBuiltinDuration(double seconds)
  {
    builtin_interfaces::msg::Duration d;
    if (seconds < 0) seconds = 0;
    const auto whole = static_cast<int32_t>(std::floor(seconds));
    const auto frac  = seconds - static_cast<double>(whole);
    d.sec = whole;
    d.nanosec = static_cast<uint32_t>(std::round(frac * 1e9));
    return d;
  }

  bool sendNavigateToHover(const geometry_msgs::msg::PoseStamped & target_pose,
                           double hover_time_s)
  {
    if (!nav_client_) return false;

    act::NavigateToHover::Goal goal;
    // Action expects Pose, not PoseStamped
    goal.target_pose = target_pose.pose;
    goal.hover_time  = toBuiltinDuration(hover_time_s);

    auto gh_future = nav_client_->async_send_goal(goal);
    if (gh_future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "NavigateToHover: goal not accepted (timeout).");
      return false;
    }
    auto goal_handle = gh_future.get();
    if (!goal_handle) {
      RCLCPP_WARN(get_logger(), "NavigateToHover: goal rejected.");
      return false;
    }

    auto result_future = nav_client_->async_get_result(goal_handle);
    const int wait_ms = static_cast<int>((std::max(3.0, hover_time_s + 5.0)) * 1000.0);
    if (result_future.wait_for(std::chrono::milliseconds(wait_ms)) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "NavigateToHover: result timeout.");
      return false;
    }
    auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(get_logger(), "NavigateToHover: result not success (code=%d).",
                  static_cast<int>(wrapped_result.code));
      return false;
    }
    return true;
  }

  bool sendMoveForward(double dx, double dy, double speed)
  {
    if (!fwd_client_) return false;

    act::MoveForward::Goal goal;
    goal.distance_x = dx;
    goal.distance_y = dy;
    goal.speed = speed;

    auto gh_future = fwd_client_->async_send_goal(goal);
    if (gh_future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "MoveForward: goal not accepted (timeout).");
      return false;
    }
    auto goal_handle = gh_future.get();
    if (!goal_handle) {
      RCLCPP_WARN(get_logger(), "MoveForward: goal rejected.");
      return false;
    }

    const double min_time = std::max(1.0, std::hypot(dx, dy) / std::max(0.1, speed));
    auto result_future = fwd_client_->async_get_result(goal_handle);
    const int wait_ms = static_cast<int>((min_time + 5.0) * 1000.0);
    if (result_future.wait_for(std::chrono::milliseconds(wait_ms)) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "MoveForward: result timeout.");
      return false;
    }
    auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(get_logger(), "MoveForward: result not success (code=%d).",
                  static_cast<int>(wrapped_result.code));
      return false;
    }
    return true;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SurveillancePatrolClient>());
  rclcpp::shutdown();
  return 0;
}
