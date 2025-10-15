#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "crazyflie_servers/action/navigate_to_hover.hpp"
#include "crazyflie_servers/action/move_forward.hpp"

using NavigateToHover = crazyflie_servers::action::NavigateToHover;
using MoveForward     = crazyflie_servers::action::MoveForward;
using namespace std::chrono_literals;

class DualActionClient : public rclcpp::Node {
public:
  DualActionClient() : rclcpp::Node("dual_action_client") {
    // Parameters
    ns_            = declare_parameter<std::string>("ns", "drone1");
    target_z_      = declare_parameter<double>("target_z", 10.0);
    yaw_           = declare_parameter<double>("yaw", 0.0);
    hover_sec_     = declare_parameter<int>("hover_sec", 2);
    distance_x_    = declare_parameter<double>("distance_x", 2.0);
    distance_y_    = declare_parameter<double>("distance_y", 10.0);

    speed_         = declare_parameter<double>("speed", 0.25);
    navigate_action_ = declare_parameter<std::string>("navigate_action", "/" + ns_ + "/navigate_to_hover");
    forward_action_  = declare_parameter<std::string>("forward_action",  "/" + ns_ + "/move_forward");

    RCLCPP_INFO(get_logger(),
      "DualActionClient ns='%s'\n  navigate_action=%s\n  forward_action=%s\n  target_z=%.2f yaw=%.2f hover=%ds\n  distance_x=%.2f speed=%.2f",
      ns_.c_str(), navigate_action_.c_str(), forward_action_.c_str(),
      target_z_, yaw_, hover_sec_, distance_x_, distance_y_, speed_);

    nav_client_ = rclcpp_action::create_client<NavigateToHover>(this, navigate_action_);
    fwd_client_ = rclcpp_action::create_client<MoveForward>(this, forward_action_);

    timer_ = create_wall_timer(250ms, std::bind(&DualActionClient::start_sequence, this));
  }

private:
  // params
  std::string ns_, navigate_action_, forward_action_;
  double target_z_, yaw_;
  int    hover_sec_;
  double distance_x_, distance_y_, speed_;

  // clients
  rclcpp_action::Client<NavigateToHover>::SharedPtr nav_client_;
  rclcpp_action::Client<MoveForward>::SharedPtr     fwd_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void start_sequence() {
    timer_->cancel();

    RCLCPP_INFO(get_logger(), "Waiting for action servers...");
    if (!nav_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "NavigateToHover server not available at %s", navigate_action_.c_str());
      rclcpp::shutdown(); return;
    }
    if (!fwd_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "MoveForward server not available at %s", forward_action_.c_str());
      rclcpp::shutdown(); return;
    }
    RCLCPP_INFO(get_logger(), "Servers ready. Starting mission...");

    // Build + send hover goal
    NavigateToHover::Goal nav_goal;
    nav_goal.target_pose.position.x = 0.0;
    nav_goal.target_pose.position.y = 0.0;
    nav_goal.target_pose.position.z = target_z_;
    nav_goal.target_pose.orientation.x = 0.0;
    nav_goal.target_pose.orientation.y = 0.0;
    nav_goal.target_pose.orientation.z = 0.0;
    nav_goal.target_pose.orientation.w = 1.0;
    nav_goal.yaw = yaw_;
    nav_goal.hover_time.sec = hover_sec_;
    nav_goal.hover_time.nanosec = 0;

    auto nav_opts = rclcpp_action::Client<NavigateToHover>::SendGoalOptions();
    nav_opts.feedback_callback =
      [this](auto, const std::shared_ptr<const NavigateToHover::Feedback> fb) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "[%s] Hover FB: remaining=%.2fs  dist_to_target=%.2f",
          ns_.c_str(), fb->remaining_hover_s, fb->distance_to_target);
      };
    nav_opts.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavigateToHover>::WrappedResult &res) {
        if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "[%s] Hover result: success=true msg='%s'",
                      ns_.c_str(), res.result->message.c_str());
          send_forward_goal();
        } else {
          RCLCPP_ERROR(this->get_logger(), "[%s] Hover failed (code=%d) msg='%s'",
                       ns_.c_str(), static_cast<int>(res.code),
                       res.result ? res.result->message.c_str() : "");
          rclcpp::shutdown();
        }
      };

    RCLCPP_INFO(get_logger(), "[%s] Sending hover: z=%.2f yaw=%.2f hover=%ds",
                ns_.c_str(), target_z_, yaw_, hover_sec_);
    nav_client_->async_send_goal(nav_goal, nav_opts);
  }

  void send_forward_goal() {
    MoveForward::Goal fwd_goal;
    fwd_goal.distance_x = distance_x_;
    fwd_goal.distance_y = distance_y_;
    fwd_goal.speed      = speed_;

    auto fwd_opts = rclcpp_action::Client<MoveForward>::SendGoalOptions();
    fwd_opts.feedback_callback =
      [this](auto, const std::shared_ptr<const MoveForward::Feedback> fb) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "[%s] Forward FB: rem_x=%.2f rem_y=%.2f rem_total=%.2f",
          ns_.c_str(), fb->remaining_x, fb->remaining_y, fb->remaining_total);
      };
    fwd_opts.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<MoveForward>::WrappedResult &res) {
        const bool ok = (res.code == rclcpp_action::ResultCode::SUCCEEDED);
        RCLCPP_INFO(this->get_logger(), "[%s] Forward result: success=%s msg='%s'",
                    ns_.c_str(), ok ? "true" : "false",
                    res.result ? res.result->message.c_str() : "");
        rclcpp::shutdown();
      };

    RCLCPP_INFO(get_logger(), "[%s] Sending forward: distance_x=%.2f speed=%.2f",
                ns_.c_str(), distance_x_, speed_);
    fwd_client_->async_send_goal(fwd_goal, fwd_opts);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualActionClient>());
  rclcpp::shutdown();
  return 0;
}

