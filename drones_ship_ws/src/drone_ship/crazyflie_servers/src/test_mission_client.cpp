#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "crazyflie_servers/action/navigate_to_hover.hpp"
#include "crazyflie_servers/action/move_forward.hpp"

using namespace std::chrono_literals;

class MissionClient : public rclcpp::Node {
public:
  using NavigateToHover = crazyflie_servers::action::NavigateToHover;
  using MoveForward = crazyflie_servers::action::MoveForward;

  using HoverGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToHover>;
  using MoveGoalHandle  = rclcpp_action::ClientGoalHandle<MoveForward>;

  MissionClient() : Node("mission_client") {
    hover_client_ = rclcpp_action::create_client<NavigateToHover>(
      this, "/drone1/navigate_to_hover");
    move_client_ = rclcpp_action::create_client<MoveForward>(
      this, "/drone1/move_forward");

    // Wait for servers
    RCLCPP_INFO(get_logger(), "Waiting for action servers...");
    hover_client_->wait_for_action_server();
    move_client_->wait_for_action_server();
    RCLCPP_INFO(get_logger(), "Servers ready. Starting mission...");

    // Step 1: ascend
    send_hover_goal();
  }

private:
  rclcpp_action::Client<NavigateToHover>::SharedPtr hover_client_;
  rclcpp_action::Client<MoveForward>::SharedPtr move_client_;

  void send_hover_goal() {
    NavigateToHover::Goal goal;
    goal.target_pose.position.x = 0.0;
    goal.target_pose.position.y = 0.0;
    goal.target_pose.position.z = 10.0;  // ascend 10 m
    goal.yaw = 0.0;
    goal.hover_time.sec = 2;
    goal.hover_time.nanosec = 0;

    RCLCPP_INFO(get_logger(), "Sending hover goal (10 m up)...");
    auto opts = rclcpp_action::Client<NavigateToHover>::SendGoalOptions();
    opts.result_callback = [this](const HoverGoalHandle::WrappedResult &res) {
      if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Step 1: Ascend done ✅");
        send_forward_goal();
      } else {
        RCLCPP_ERROR(get_logger(), "Hover failed or canceled");
      }
    };
    hover_client_->async_send_goal(goal, opts);
  }

  void send_forward_goal() {
    MoveForward::Goal goal;
    goal.distance_x = 10.0;  // move forward 10 m
    goal.distance_y = 0.0;
    goal.speed = 0.25;

    RCLCPP_INFO(get_logger(), "Sending forward move goal...");
    auto opts = rclcpp_action::Client<MoveForward>::SendGoalOptions();
    opts.feedback_callback = [this](MoveGoalHandle::SharedPtr,
                                    const std::shared_ptr<const MoveForward::Feedback> fb) {
      RCLCPP_INFO(get_logger(), "Forward progress: rem=%.2f m", fb->remaining_total);
    };
    opts.result_callback = [this](const MoveGoalHandle::WrappedResult &res) {
      if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Step 2: Forward motion done ✅");
        send_left_goal();
      } else {
        RCLCPP_ERROR(get_logger(), "Forward goal failed or canceled");
      }
    };
    move_client_->async_send_goal(goal, opts);
  }

  void send_left_goal() {
    MoveForward::Goal goal;
    goal.distance_x = 0.0;
    goal.distance_y = 5.0;  // move left 5 m (positive Y)
    goal.speed = 0.25;

    RCLCPP_INFO(get_logger(), "Sending left move goal...");
    auto opts = rclcpp_action::Client<MoveForward>::SendGoalOptions();
    opts.feedback_callback = [this](MoveGoalHandle::SharedPtr,
                                    const std::shared_ptr<const MoveForward::Feedback> fb) {
      RCLCPP_INFO(get_logger(), "Lateral progress: rem=%.2f m", fb->remaining_total);
    };
    opts.result_callback = [this](const MoveGoalHandle::WrappedResult &res) {
      if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "✅ Mission complete!");
        rclcpp::shutdown();
      } else {
        RCLCPP_ERROR(get_logger(), "Lateral move failed or canceled");
      }
    };
    move_client_->async_send_goal(goal, opts);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionClient>());
  rclcpp::shutdown();
  return 0;
}
