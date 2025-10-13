// explore_and_track_controller.cpp
// ROS 2 Jazzy — Orchestrator with two action servers (Explore, Track) + pause/resume services.
// Motion is delegated to low-level actions: /navigate_to_hover and /move_forward.
// Cancel of /explore preempts any in-flight low-level forward/hover goals.

// Optional: enable low-level tracker delegation if you have TrackTarget.action
// #define HAVE_TRACK_TARGET 1

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <future>
#include <chrono>

#include "crazyflie_yolo/action/explore.hpp"
#include "crazyflie_yolo/action/track.hpp"
#include "crazyflie_yolo/action/navigate_to_hover.hpp"
#include "crazyflie_yolo/action/move_forward.hpp"

#ifdef HAVE_TRACK_TARGET
  #include "crazyflie_yolo/action/track_target.hpp"
#endif

using namespace std::chrono_literals;

using Explore          = crazyflie_yolo::action::Explore;
using Track            = crazyflie_yolo::action::Track;
using NavigateToHover  = crazyflie_yolo::action::NavigateToHover;
using MoveForward      = crazyflie_yolo::action::MoveForward;

#ifdef HAVE_TRACK_TARGET
  using TrackTargetLow   = crazyflie_yolo::action::TrackTarget;
#endif

class ExploreAndTrackController : public rclcpp::Node {
public:
  using ExploreHandle   = rclcpp_action::ServerGoalHandle<Explore>;
  using TrackHandle     = rclcpp_action::ServerGoalHandle<Track>;
  using HoverHandle     = rclcpp_action::ClientGoalHandle<NavigateToHover>;
  using ForwardHandle   = rclcpp_action::ClientGoalHandle<MoveForward>;
#ifdef HAVE_TRACK_TARGET
  using TrackLowHandle  = rclcpp_action::ClientGoalHandle<TrackTargetLow>;
#endif

  ExploreAndTrackController() : rclcpp::Node("explore_and_track_controller")
  {
    // Parameters
    nav_action_name_  = declare_parameter<std::string>("navigate_action", "/navigate_to_hover");
    fwd_action_name_  = declare_parameter<std::string>("forward_action",  "/move_forward");
    low_track_action_ = declare_parameter<std::string>("low_tracker_action", "/track_target");

    default_alt_      = declare_parameter<double>("default_ascend_altitude", 10.0);
    default_leg_m_    = declare_parameter<double>("default_leg_distance",    20.0);
    default_speed_    = declare_parameter<double>("default_forward_speed",   0.25);
    default_yaw_rad_  = declare_parameter<double>("default_yaw",             0.0);
    hover_sec_        = declare_parameter<int>("hover_sec", 2);

    use_low_tracker_  = declare_parameter<bool>("use_low_tracker", false);

    // Low-level action clients
    hover_client_     = rclcpp_action::create_client<NavigateToHover>(this, nav_action_name_);
    forward_client_   = rclcpp_action::create_client<MoveForward>(this, fwd_action_name_);
#ifdef HAVE_TRACK_TARGET
    if (use_low_tracker_) {
      low_track_client_ = rclcpp_action::create_client<TrackTargetLow>(this, low_track_action_);
    }
#endif

    // Services: pause/resume
    pause_srv_ = create_service<std_srvs::srv::Trigger>(
      "pause_exploration",
      std::bind(&ExploreAndTrackController::onPause, this, std::placeholders::_1, std::placeholders::_2));

    resume_srv_ = create_service<std_srvs::srv::Trigger>(
      "resume_exploration",
      std::bind(&ExploreAndTrackController::onResume, this, std::placeholders::_1, std::placeholders::_2));

    // Action servers
    explore_server_ = rclcpp_action::create_server<Explore>(
      this, "explore",
      std::bind(&ExploreAndTrackController::handle_explore_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ExploreAndTrackController::handle_explore_cancel,   this, std::placeholders::_1),
      std::bind(&ExploreAndTrackController::handle_explore_accepted, this, std::placeholders::_1));

    track_server_ = rclcpp_action::create_server<Track>(
      this, "track",
      std::bind(&ExploreAndTrackController::handle_track_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ExploreAndTrackController::handle_track_cancel,   this, std::placeholders::_1),
      std::bind(&ExploreAndTrackController::handle_track_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "Explore&Track controller up: actions [/explore, /track], services [/pause_exploration, /resume_exploration]");
  }

private:
  // ===== Params =====
  std::string nav_action_name_, fwd_action_name_, low_track_action_;
  double default_alt_{10.0}, default_leg_m_{20.0}, default_speed_{0.25}, default_yaw_rad_{0.0};
  int hover_sec_{2};
  bool use_low_tracker_{false};

  // ===== Clients =====
  rclcpp_action::Client<NavigateToHover>::SharedPtr hover_client_;
  rclcpp_action::Client<MoveForward>::SharedPtr     forward_client_;
#ifdef HAVE_TRACK_TARGET
  rclcpp_action::Client<TrackTargetLow>::SharedPtr  low_track_client_;
#endif

  // ===== Servers & Services =====
  rclcpp_action::Server<Explore>::SharedPtr explore_server_;
  rclcpp_action::Server<Track>::SharedPtr   track_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_, resume_srv_;

  // ===== State & Synchronization =====
  std::mutex mtx_;                  // pause/stop flags
  std::condition_variable cv_;
  bool paused_{false};
  bool stop_explore_thread_{false};

  std::mutex fwd_mtx_;              // in-flight forward handle
  ForwardHandle::SharedPtr current_forward_handle_{};

  std::mutex hov_mtx_;              // in-flight hover handle
  HoverHandle::SharedPtr   current_hover_handle_{};

  // ---------- Explore server handlers ----------
  rclcpp_action::GoalResponse handle_explore_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const Explore::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Explore goal: alt=%.1f m, leg=%.1f m, speed=%.2f m/s, yaw=%.2f rad",
                goal->ascend_altitude, goal->leg_distance, goal->forward_speed, goal->yaw);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_explore_cancel(const std::shared_ptr<ExploreHandle>)
  {
    RCLCPP_WARN(get_logger(), "Explore: cancel requested → preempting low-level goals");
    {
      std::scoped_lock lk(mtx_);
      stop_explore_thread_ = true;
      paused_ = false;
    }
    cv_.notify_all();

    // Cancel in-flight forward
    {
      std::scoped_lock lk(fwd_mtx_);
      if (current_forward_handle_) {
        forward_client_->async_cancel_goal(current_forward_handle_);
      }
    }
    // Cancel in-flight hover
    {
      std::scoped_lock lk(hov_mtx_);
      if (current_hover_handle_) {
        hover_client_->async_cancel_goal(current_hover_handle_);
      }
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_explore_accepted(const std::shared_ptr<ExploreHandle> gh)
  {
    std::thread(&ExploreAndTrackController::explore_execute, this, gh).detach();
  }

  void explore_execute(const std::shared_ptr<ExploreHandle> gh)
  {
    auto feedback = std::make_shared<Explore::Feedback>();
    auto result   = std::make_shared<Explore::Result>();

    // Wait for low-level servers
    if (!hover_client_->wait_for_action_server(5s) || !forward_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "Low-level hover/forward servers unavailable");
      result->success = false; result->message = "missing low-level servers";
      gh->abort(result);
      return;
    }

    const auto goal = gh->get_goal();
    const double ascend_alt = goal->ascend_altitude > 0.0 ? goal->ascend_altitude : default_alt_;
    const double leg_m      = goal->leg_distance    > 0.0 ? goal->leg_distance    : default_leg_m_;
    const double fwd_speed  = goal->forward_speed   > 0.0 ? goal->forward_speed   : default_speed_;
    const double yaw_rad    = goal->yaw;

    // 1) Ascend & hover
    if (gh->is_canceling()) {
      result->success = false; result->message = "canceled before start";
      gh->canceled(result); return;
    }
    {
      NavigateToHover::Goal h;
      h.target_pose.position.x = 0.0;
      h.target_pose.position.y = 0.0;
      h.target_pose.position.z = ascend_alt;
      h.yaw = yaw_rad;
      h.hover_time.sec = hover_sec_;
      h.hover_time.nanosec = 0;

      auto send_opts = rclcpp_action::Client<NavigateToHover>::SendGoalOptions();
      std::promise<rclcpp_action::ResultCode> p; auto fut = p.get_future();
      send_opts.result_callback = [&p](const HoverHandle::WrappedResult& r){ p.set_value(r.code); };

      auto future_handle = hover_client_->async_send_goal(h, send_opts);
      if (future_handle.wait_for(3s) != std::future_status::ready) {
        result->success = false; result->message = "hover goal not accepted";
        gh->abort(result); return;
      }
      {
        std::scoped_lock lk(hov_mtx_);
        current_hover_handle_ = future_handle.get();
      }

      // Wait for hover result (while honoring cancel)
      auto wait_start = this->now();
      while (rclcpp::ok()) {
        if (gh->is_canceling() || stop_explore_thread_) {
          // cancel low-level hover
          std::scoped_lock lk(hov_mtx_);
          if (current_hover_handle_) hover_client_->async_cancel_goal(current_hover_handle_);
          result->success = false; result->message = "explore canceled during hover";
          gh->canceled(result); return;
        }
        if (fut.wait_for(50ms) == std::future_status::ready) break;
      }
      {
        std::scoped_lock lk(hov_mtx_);
        current_hover_handle_.reset();
      }
      if (fut.get() != rclcpp_action::ResultCode::SUCCEEDED) {
        result->success = false; result->message = "ascend/hover failed";
        gh->abort(result); return;
      }
    }

    // 2) Repeating forward legs until canceled; pause between legs when requested
    stop_explore_thread_ = false;
    while (rclcpp::ok()) {
      if (gh->is_canceling() || stop_explore_thread_) {
        result->success = false; result->message = "explore canceled";
        gh->canceled(result); return;
      }

      // Pause handling — wait here between legs
      {
        std::unique_lock lk(mtx_);
        cv_.wait(lk, [this]{ return !paused_ || stop_explore_thread_; });
        if (stop_explore_thread_) {
          result->success = false; result->message = "stopped";
          gh->canceled(result); return;
        }
      }

      // Send one forward leg
      MoveForward::Goal f; f.distance_x = leg_m; f.distance_y = 0.0;f.speed = fwd_speed;

      auto opts = rclcpp_action::Client<MoveForward>::SendGoalOptions();
      opts.feedback_callback = [this, gh](ForwardHandle::SharedPtr,
        const std::shared_ptr<const MoveForward::Feedback> fb){
          auto fb_out = std::make_shared<Explore::Feedback>();
          fb_out->remaining_in_leg = fb->remaining_total;
          gh->publish_feedback(fb_out);
        };
      std::promise<rclcpp_action::ResultCode> p_res; auto fut_res = p_res.get_future();
      opts.result_callback = [&p_res](const ForwardHandle::WrappedResult& r){ p_res.set_value(r.code); };

      auto future_handle = forward_client_->async_send_goal(f, opts);
      if (future_handle.wait_for(3s) != std::future_status::ready) {
        result->success = false; result->message = "forward goal not accepted";
        gh->abort(result); return;
      }
      {
        std::scoped_lock lk(fwd_mtx_);
        current_forward_handle_ = future_handle.get();
      }

      // Wait for the forward result; honor cancel requests
      while (rclcpp::ok()) {
        if (gh->is_canceling() || stop_explore_thread_) {
          // Cancel in-flight forward goal
          std::scoped_lock lk(fwd_mtx_);
          if (current_forward_handle_) forward_client_->async_cancel_goal(current_forward_handle_);
          result->success = false; result->message = "explore canceled during forward";
          gh->canceled(result); return;
        }
        if (fut_res.wait_for(50ms) == std::future_status::ready) break;
      }
      {
        std::scoped_lock lk(fwd_mtx_);
        current_forward_handle_.reset();
      }
      if (fut_res.get() != rclcpp_action::ResultCode::SUCCEEDED) {
        // aborted/canceled → end explore as aborted
        result->success = false; result->message = "forward leg failed";
        gh->abort(result); return;
      }

      // Loop continues: next leg unless paused/canceled
    }
  }

  // ---------- Track server handlers ----------
  rclcpp_action::GoalResponse handle_track_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const Track::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Track goal: target_id=%d note='%s'", goal->target_id, goal->note.c_str());
    // Auto-pause exploration when a track goal arrives
    {
      std::scoped_lock lk(mtx_);
      paused_ = true;
    }
    cv_.notify_all();
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_track_cancel(const std::shared_ptr<TrackHandle>)
  {
    RCLCPP_WARN(get_logger(), "Track: cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_track_accepted(const std::shared_ptr<TrackHandle> gh)
  {
    std::thread(&ExploreAndTrackController::track_execute, this, gh).detach();
  }

  void track_execute(const std::shared_ptr<TrackHandle> gh)
  {
    auto result = std::make_shared<Track::Result>();

#ifdef HAVE_TRACK_TARGET
    if (use_low_tracker_ && low_track_client_) {
      if (!low_track_client_->wait_for_action_server(3s)) {
        RCLCPP_WARN(get_logger(), "Low-level tracker missing, holding instead");
      } else {
        TrackTargetLow::Goal t; t.target_id = gh->get_goal()->target_id;
        std::promise<rclcpp_action::ResultCode> p; auto fut = p.get_future();
        auto opts = rclcpp_action::Client<TrackTargetLow>::SendGoalOptions();
        opts.result_callback = [&p](const TrackLowHandle::WrappedResult& r){ p.set_value(r.code); };
        low_track_client_->async_send_goal(t, opts);
        auto code = fut.get();
        if (code == rclcpp_action::ResultCode::SUCCEEDED) {
          result->success = true; result->message = "tracking complete";
          gh->succeed(result); return;
        } else {
          result->success = false; result->message = "low-level tracking failed";
          gh->abort(result); return;
        }
      }
    }
#endif

    // Hold mode: just stay in TRACK until the goal is canceled
    RCLCPP_INFO(get_logger(), "Tracking (hold mode). Cancel the track goal to finish.");
    rclcpp::Rate r(10.0);
    while (rclcpp::ok()) {
      if (gh->is_canceling()) {
        result->success = false; result->message = "track canceled";
        gh->canceled(result); return;
      }
      r.sleep();
    }
  }

  // ---------- Services ----------
  void onPause(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
    {
      std::scoped_lock lk(mtx_);
      paused_ = true;
    }
    cv_.notify_all();
    resp->success = true; resp->message = "exploration paused";
    RCLCPP_INFO(get_logger(), "Exploration paused by service call");
  }

  void onResume(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
    {
      std::scoped_lock lk(mtx_);
      paused_ = false;
      stop_explore_thread_ = false;
    }
    cv_.notify_all();
    resp->success = true; resp->message = "exploration resumed";
    RCLCPP_INFO(get_logger(), "Exploration resumed by service call");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExploreAndTrackController>());
  rclcpp::shutdown();
  return 0;
}
