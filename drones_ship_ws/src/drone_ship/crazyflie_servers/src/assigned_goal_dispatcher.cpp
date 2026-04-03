#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "crazyflie_yolo/msg/tracked_obstacle.hpp"

#include "crazyflie_servers/action/navigate_to_hover.hpp"
#include "crazyflie_servers/action/move_forward.hpp"

using namespace std::chrono_literals;

namespace msgs = crazyflie_yolo::msg;
namespace act  = crazyflie_servers::action;

struct DroneConfig {
  std::string ns;          // "/drone1"
  double target_z = 10.0;  // hover height (world z)
  double hover_sec = 2.0;  // dwell time
  double yaw = 0.0;        // reserved (not used in this simple hover)
  double dx = 10.0;        // after-hover forward X (per your server semantics)
  double dy = -10.0;       // after-hover forward Y
  double speed = 0.2;      // move speed
};

class AssignedGoalDispatcher : public rclcpp::Node {
public:
  AssignedGoalDispatcher() : Node("assigned_goal_dispatcher")
  {
    // Params (arrays must have up to 3 elements)
    drone_ns_ = this->declare_parameter<std::vector<std::string>>(
      "drone_ns", {"/drone1","/drone2","/drone3"});

    target_z_  = this->declare_parameter<std::vector<double>>("target_z",  {10.0,10.0,10.0});
    hover_sec_ = this->declare_parameter<std::vector<double>>("hover_sec", {2.0,2.0,2.0});
    yaw_       = this->declare_parameter<std::vector<double>>("yaw",       {0.0,0.0,0.0});
    distance_x_= this->declare_parameter<std::vector<double>>("distance_x",{10.0,10.0,10.0});
    distance_y_= this->declare_parameter<std::vector<double>>("distance_y",{-10.0,-10.0,-10.0});
    speed_     = this->declare_parameter<std::vector<double>>("speed",     {0.20,0.20,0.20});

    normalize(drone_ns_,  std::string("/drone1"));
    normalize(target_z_,  10.0);
    normalize(hover_sec_, 2.0);
    normalize(yaw_,       0.0);
    normalize(distance_x_,10.0);
    normalize(distance_y_,-10.0);
    normalize(speed_,     0.2);

    const int N = 3;
    handlers_.reserve(N);
    for (int i = 0; i < N; ++i) {
      DroneConfig cfg;
      cfg.ns       = drone_ns_[i];
      cfg.target_z = target_z_[i];
      cfg.hover_sec= hover_sec_[i];
      cfg.yaw      = yaw_[i];
      cfg.dx       = distance_x_[i];
      cfg.dy       = distance_y_[i];
      cfg.speed    = speed_[i];

      handlers_.push_back(std::make_shared<DroneHandler>(shared_from_this(), cfg));
    }

    RCLCPP_INFO(get_logger(), "assigned_goal_dispatcher ready:");
    for (auto &h : handlers_) {
      RCLCPP_INFO(get_logger(),
        "  ns=%s | z=%.1f hover=%.1fs yaw=%.1f dx=%.1f dy=%.1f v=%.2f",
        h->cfg.ns.c_str(), h->cfg.target_z, h->cfg.hover_sec, h->cfg.yaw,
        h->cfg.dx, h->cfg.dy, h->cfg.speed);
    }
  }

private:
  template<typename T>
  void normalize(std::vector<T> &v, const T &fill, std::size_t N=3) {
    if (v.size() < N) v.resize(N, fill);
    if (v.size() > N) v.resize(N);
  }

  struct DroneHandler {
    DroneHandler(const rclcpp::Node::SharedPtr & node, const DroneConfig & cfg_in)
    : node(node), cfg(cfg_in), logger(node->get_logger())
    {
      // Sub: /droneN/assigned_obstacle
      sub = node->create_subscription<msgs::TrackedObstacle>(
        cfg.ns + "/assigned_obstacle", 10,
        [this](const msgs::TrackedObstacle & msg){ onAssigned(msg); });

      // Action clients in that namespace
      nav_client = rclcpp_action::create_client<act::NavigateToHover>(
        node, cfg.ns + "/navigate_to_hover");
      fwd_client = rclcpp_action::create_client<act::MoveForward>(
        node, cfg.ns + "/move_forward");

      // Non-blocking periodic wait for servers
      wait_timer = node->create_wall_timer(500ms, [this](){
        if (!nav_ready) {
          nav_ready = nav_client->wait_for_action_server(0s);
          if (!nav_ready) {
            RCLCPP_INFO(logger, "[%s] waiting for navigate_to_hover server...", cfg.ns.c_str());
          }
        }
        if (!fwd_ready) {
          fwd_ready = fwd_client->wait_for_action_server(0s);
          if (!fwd_ready) {
            RCLCPP_INFO(logger, "[%s] waiting for move_forward server...", cfg.ns.c_str());
          }
        }
        if (nav_ready && fwd_ready) {
          RCLCPP_INFO(logger, "[%s] action servers ready.", cfg.ns.c_str());
          wait_timer->cancel();
        }
      });
    }

    void onAssigned(const msgs::TrackedObstacle & msg)
    {
      if (!nav_ready || !fwd_ready) return;      // servers not ready yet
      if (msg.id < 0) return;                    // no assignment

      // Debounce: only retrigger if new id or moved > 0.5 m
      const bool same_id = (last_target_id == msg.id);
      const double moved = std::hypot(msg.position.x - last_pos_x,
                                      msg.position.y - last_pos_y);
      if (same_id && moved < 0.5) return;

      last_target_id = msg.id;
      last_pos_x = msg.position.x;
      last_pos_y = msg.position.y;

      // 1) Hover over obstacle at cfg.target_z
      geometry_msgs::msg::Pose hover_pose;
      hover_pose.position.x = msg.position.x;
      hover_pose.position.y = msg.position.y;
      hover_pose.position.z = cfg.target_z;
      hover_pose.orientation.w = 1.0;

      RCLCPP_INFO(logger,
        "[%s] Assigned id=%d -> Hover(%.2f, %.2f, %.2f) then Move(dx=%.2f, dy=%.2f, v=%.2f)",
        cfg.ns.c_str(), msg.id,
        hover_pose.position.x, hover_pose.position.y, hover_pose.position.z,
        cfg.dx, cfg.dy, cfg.speed);

      act::NavigateToHover::Goal nav_goal;
      nav_goal.target_pose = hover_pose;                                 // NOTE: Pose (not PoseStamped)
      nav_goal.hover_time  = rclcpp::Duration::from_seconds(cfg.hover_sec);

      auto nav_opts = rclcpp_action::Client<act::NavigateToHover>::SendGoalOptions{};
      auto nav_fut  = nav_client->async_send_goal(nav_goal, nav_opts);

      // Chain MoveForward in a background thread
      std::thread([this, nav_fut = std::move(nav_fut)]() mutable {
        // Goal handle
        if (nav_fut.wait_for(10s) != std::future_status::ready) {
          RCLCPP_WARN(logger, "[%s] NavigateToHover goal not accepted (timeout)", cfg.ns.c_str());
          return;
        }
        auto gh = nav_fut.get();
        if (!gh) {
          RCLCPP_WARN(logger, "[%s] NavigateToHover goal rejected", cfg.ns.c_str());
          return;
        }
        // Result
        auto res_fut = nav_client->async_get_result(gh);
        if (res_fut.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
          RCLCPP_WARN(logger, "[%s] NavigateToHover result timeout", cfg.ns.c_str());
          return;
        }
        auto res = res_fut.get();
        if (res.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_WARN(logger, "[%s] NavigateToHover did not succeed (code=%d)",
                      cfg.ns.c_str(), (int)res.code);
          return;
        }

        // 2) MoveForward
        act::MoveForward::Goal fwd_goal;
        fwd_goal.distance_x = cfg.dx;
        fwd_goal.distance_y = cfg.dy;
        fwd_goal.speed      = cfg.speed;

        auto fwd_opts  = rclcpp_action::Client<act::MoveForward>::SendGoalOptions{};
        auto fwd_gh_fut= fwd_client->async_send_goal(fwd_goal, fwd_opts);
        if (fwd_gh_fut.wait_for(5s) != std::future_status::ready) {
          RCLCPP_WARN(logger, "[%s] MoveForward goal not accepted (timeout)", cfg.ns.c_str());
          return;
        }
        auto fwd_gh = fwd_gh_fut.get();
        if (!fwd_gh) {
          RCLCPP_WARN(logger, "[%s] MoveForward goal rejected", cfg.ns.c_str());
          return;
        }
        // Result (time budget based on distance/speed)
        const double min_time = std::max(1.0, std::hypot(cfg.dx, cfg.dy) / std::max(0.05, cfg.speed));
        auto fwd_res_fut = fwd_client->async_get_result(fwd_gh);
        if (fwd_res_fut.wait_for(std::chrono::milliseconds((int)((min_time+10.0)*1000.0))) !=
            std::future_status::ready) {
          RCLCPP_WARN(logger, "[%s] MoveForward result timeout", cfg.ns.c_str());
          return;
        }
        auto fwd_res = fwd_res_fut.get();
        if (fwd_res.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_WARN(logger, "[%s] MoveForward did not succeed (code=%d)",
                      cfg.ns.c_str(), (int)fwd_res.code);
          return;
        }
        RCLCPP_INFO(logger, "[%s] Sequence complete for target id=%d", cfg.ns.c_str(), last_target_id);
      }).detach();
    }

    // Per-drone state
    rclcpp::Node::SharedPtr node;
    rclcpp::Logger logger;
    DroneConfig cfg;

    rclcpp::Subscription<msgs::TrackedObstacle>::SharedPtr sub;
    rclcpp_action::Client<act::NavigateToHover>::SharedPtr nav_client;
    rclcpp_action::Client<act::MoveForward>::SharedPtr     fwd_client;

    rclcpp::TimerBase::SharedPtr wait_timer;
    bool nav_ready{false};
    bool fwd_ready{false};

    int    last_target_id{-1};
    double last_pos_x{std::numeric_limits<double>::quiet_NaN()};
    double last_pos_y{std::numeric_limits<double>::quiet_NaN()};
  };

  // Params
  std::vector<std::string> drone_ns_;
  std::vector<double> target_z_, hover_sec_, yaw_, distance_x_, distance_y_, speed_;

  // Per-drone handlers
  std::vector<std::shared_ptr<DroneHandler>> handlers_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AssignedGoalDispatcher>());
  rclcpp::shutdown();
  return 0;
}
