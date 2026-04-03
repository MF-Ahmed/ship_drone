// assignment_node.cpp
// UPDATED (2026-01-09):
//   - Global DONE lockout by container identity (class_id)
//     => If any drone declares DONE on class_id=c, then NO drone can be assigned ANY fused track with class_id=c.
//   - Keeps per-drone SURVEILLANCE latch behavior as-is (requires reset to re-enable tracking for that drone).
//
// Notes:
// - Your fused tracks array order can change each callback. Stickiness MUST be based on track_id (trk.id),
//   NOT the track index.
// - GT mapping assumes class_id is 0..4 for container1..5, so container_id = class_id + 1.
//
// IMPORTANT behavioral change:
// - Previously: done_target_ids_ blocked by track_id only.
// - Now: done_class_ids_ blocks globally by class_id (container identity).

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Dense>

#include <unordered_map>
#include <unordered_set>
#include <regex>
#include <cmath>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <limits>
#include <queue>
#include <tuple>
#include <algorithm>
#include <vector>
#include <filesystem>

#include "crazyflie_yolo/msg/tracked_obstacle.hpp"
#include "crazyflie_yolo/msg/tracked_obstacle_array.hpp"
#include "crazyflie_yolo/msg/assign_target.hpp"
#include "crazyflie_yolo/msg/assignment_metrics.hpp"

namespace msgs = crazyflie_yolo::msg;
using rclcpp::QoS;

static constexpr double kPi = 3.14159265358979323846;

// -----------------------------------------------------------------------------
// Helper structs
// -----------------------------------------------------------------------------
struct DroneState {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double yaw = 0.0; // ENU yaw
};

struct TargetTrack {
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();           // world/ENU
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity(); // 3x3 P
  int class_id = -1; // container identity (0..4 for container1..5)
  int id = -1;       // stable tracking id
};

// Extract "/droneX" namespace prefix from odom topic
static std::string ns_from_topic(const std::string & topic)
{
  std::smatch m;
  static const std::regex re("^(/[^/]+)");
  if (std::regex_search(topic, m, re)) return m[1].str();
  return "";
}

// Map class_id (0..4) -> container_id (1..5) for GT odom lookup
static inline int container_id_from_class_id(int class_id)
{
  if (class_id < 0) return -1;
  return class_id + 1;
}

// -----------------------------------------------------------------------------
// Min-Cost Max-Flow (successive shortest path)
// -----------------------------------------------------------------------------
class MinCostFlow {
public:
  struct Edge { int to, rev, cap, flow; double cost; };
  explicit MinCostFlow(int n): n_(n) { g_.resize(n_); }
  void addEdge(int u,int v,int cap,double cost){
    Edge a{v, (int)g_[v].size(), cap, 0,  cost};
    Edge b{u, (int)g_[u].size(), 0,   0, -cost};
    g_[u].push_back(a); g_[v].push_back(b);
  }
  std::pair<int,double> solve(int s,int t,int max_flow){
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> pot(n_, 0.0);
    int flow = 0; double cost = 0.0;
    while (flow < max_flow) {
      std::vector<double> dist(n_, INF);
      std::vector<int> pv(n_, -1), pe(n_, -1);
      using Q = std::pair<double,int>;
      std::priority_queue<Q, std::vector<Q>, std::greater<Q>> pq;
      dist[s] = 0.0; pq.emplace(0.0, s);
      while(!pq.empty()){
        auto [cd,u] = pq.top(); pq.pop();
        if (cd > dist[u]) continue;
        for (int ei=0; ei<(int)g_[u].size(); ++ei){
          const auto &e = g_[u][ei];
          if (e.flow >= e.cap) continue;
          double rc = e.cost + pot[u] - pot[e.to];
          double nd = cd + rc;
          if (nd < dist[e.to]) {
            dist[e.to] = nd; pv[e.to]=u; pe[e.to]=ei; pq.emplace(nd,e.to);
          }
        }
      }
      if (!std::isfinite(dist[t])) break;
      for (int v=0; v<n_; ++v) if (std::isfinite(dist[v])) pot[v]+=dist[v];

      int add = max_flow - flow;
      for (int v=t; v!=s; v=pv[v]){
        int u=pv[v], ei=pe[v];
        if (ei<0) { add=0; break; }
        add = std::min(add, g_[u][ei].cap - g_[u][ei].flow);
      }
      if (add<=0) break;

      double path_cost=0.0;
      for (int v=t; v!=s; v=pv[v]){
        int u=pv[v], ei=pe[v];
        auto &e=g_[u][ei]; auto &er=g_[v][e.rev];
        e.flow+=add; er.flow-=add; path_cost += e.cost*add;
      }
      flow+=add; cost+=path_cost;
    }
    return {flow,cost};
  }
  const auto& graph() const { return g_; }
private:
  int n_;
  std::vector<std::vector<Edge>> g_;
};

// -----------------------------------------------------------------------------
// Hover selection utilities
// -----------------------------------------------------------------------------
struct HoverParams {
  int    l = 12;
  double r_h = 2.0;
  double h   = 6.0;
  double r_safe = 0.5;
  double v_travel = 1.5;
  double tmax = 15.0;
  double meas_sigma0 = 0.15;
  double meas_k_range = 0.02;
  double eps_time = 1e-3;
};

static inline Eigen::Matrix3d rangeAwareR(
  const Eigen::Vector3d& sensor_pos,
  const Eigen::Vector3d& target,
  double s0, double k)
{
  const double range = (sensor_pos - target).norm();
  const double s = std::max(1e-3, (s0 + k * range));
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R(0,0)=s*s; R(1,1)=s*s; R(2,2)=s*s;
  return R;
}

static inline double dOptimalityGain(
  const Eigen::Matrix3d& P,
  const Eigen::Matrix3d& H,
  const Eigen::Matrix3d& R)
{
  Eigen::Matrix3d Pinv = P.inverse();
  Eigen::Matrix3d info = H.transpose() * R.inverse() * H;
  Eigen::Matrix3d Pp   = (Pinv + info).inverse();
  const double d1 = std::max(1e-12, P.determinant());
  const double d2 = std::max(1e-12, Pp.determinant());
  return std::log(d1) - std::log(d2);
}

static inline double travelSeconds(
  const Eigen::Vector3d& s, const Eigen::Vector3d& q, double v)
{
  return (q - s).norm() / std::max(0.05, v);
}

static inline double minSepAt(
  const Eigen::Vector3d& q,
  const std::vector<Eigen::Vector3d>& others)
{
  double m = std::numeric_limits<double>::infinity();
  for (const auto &p: others) m = std::min(m, (q - p).norm());
  return m;
}

static inline bool losOK(const Eigen::Vector3d&, const Eigen::Vector3d&) { return true; }
static inline bool geofenceOK(const Eigen::Vector3d&) { return true; }
static inline bool altitudeOK(const Eigen::Vector3d& q, double min_alt, double max_alt) {
  return q.z() >= min_alt && q.z() <= max_alt;
}
static inline bool batteryOK() { return true; }

// -----------------------------------------------------------------------------
// Assignment Node
// -----------------------------------------------------------------------------
class AssignmentNode : public rclcpp::Node
{
public:
  AssignmentNode(): rclcpp::Node("assignment_node")
  {
    // ---------------- Parameters ----------------
    odom_topics_   = declare_parameter<std::vector<std::string>>("drone_odom_topics", std::vector<std::string>{});
    alloc_rate_hz_ = declare_parameter<double>("alloc_rate_hz", 1.0);

    // cost model weights
    eta_    = declare_parameter<double>("eta",    1.0);
    beta_   = declare_parameter<double>("beta",   0.2);
    gamma_  = declare_parameter<double>("gamma",  0.3);
    rho_    = declare_parameter<double>("rho",    0.2);
    kappa_  = declare_parameter<double>("kappa", 1000.0);

    // NEW: switching hysteresis (add cost if target differs from last primary track_id)
    switch_penalty_ = declare_parameter<double>("switch_penalty", 0.0);

    // gating / feasibility
    r_safe_       = declare_parameter<double>("r_safe",       0.5);
    d_max_        = declare_parameter<double>("d_max",       25.0);

    // per-UAV capacity K
    drone_capacity_ = declare_parameter<int>("drone_capacity", 1);

    // hover policy defaults
    hover_params_.l            = declare_parameter<int>("hover_l",         12);
    hover_params_.r_h          = declare_parameter<double>("hover_radius",  2.0);
    hover_params_.h            = declare_parameter<double>("hover_height",  6.0);
    hover_params_.r_safe       = r_safe_;
    hover_params_.v_travel     = declare_parameter<double>("travel_speed",  1.5);
    hover_params_.tmax         = declare_parameter<double>("hover_Tmax",   15.0);
    hover_params_.meas_sigma0  = declare_parameter<double>("meas_sigma0",   0.15);
    hover_params_.meas_k_range = declare_parameter<double>("meas_k_range",  0.02);

    // "inside/near circle" + "best estimator" knobs
    hover_inside_tol_m_      = declare_parameter<double>("hover_inside_tol_m", 0.3);
    hover_z_tol_m_           = declare_parameter<double>("hover_z_tol_m", 0.5);
    require_best_estimator_  = declare_parameter<bool>("require_best_estimator", true);
    best_margin_logdet_      = declare_parameter<double>("best_margin_logdet", 0.05);

    min_alt_       = declare_parameter<double>("min_alt",  1.0);
    max_alt_       = declare_parameter<double>("max_alt", 50.0);

    verbose_ = declare_parameter<bool>("verbose", false);

    // termination
    tau_logdet_ = declare_parameter<double>("tau_logdet", -5.0);
    tau_dJ_     = declare_parameter<double>("tau_dJ", 1e-3);
    tmax_meas_  = declare_parameter<double>("tmax_meas", 2.0);
    print_term_ = declare_parameter<bool>("print_term", false);

    // whether to require both reach(circle) AND termination to declare done
    require_reach_and_term_ = declare_parameter<bool>("require_reach_and_term", true);

    // fixed height flight
    use_fixed_altitude_ = declare_parameter<bool>("use_fixed_altitude", true);
    fixed_altitude_m_   = declare_parameter<double>("fixed_altitude_m", 10.0);

    // ring visualization toggles
    viz_ring_  = declare_parameter<bool>("viz_ring", true);
    viz_qstar_ = declare_parameter<bool>("viz_qstar", true);

    // publish qstar_dxdy in drone frame (for action client)
    publish_qstar_dxdy_ = declare_parameter<bool>("publish_qstar_dxdy", true);

    // publish metrics
    publish_metrics_ = declare_parameter<bool>("publish_metrics", true);
    metrics_topic_   = declare_parameter<std::string>("metrics_topic", "assignment_metrics");

    // CSV logging
    csv_enable_ = declare_parameter<bool>("csv_enable", false);
    csv_path_   = declare_parameter<std::string>("csv_path", "");
    csv_append_ = declare_parameter<bool>("csv_append", false);
    csv_flush_each_tick_ = declare_parameter<bool>("csv_flush_each_tick", true);

    // -------- GT from container odometry (association by class_id) --------
    gt_enable_    = declare_parameter<bool>("gt_enable", false);
    gt_max_age_s_ = declare_parameter<double>("gt_max_age_s", 1.0);
    gt_odom_topics_ = declare_parameter<std::vector<std::string>>(
      "gt_odom_topics",
      std::vector<std::string>{
        "/container1/odometry",
        "/container2/odometry",
        "/container3/odometry",
        "/container4/odometry",
        "/container5/odometry"
      });

    // -------- PER-DRONE printing --------
    print_enable_      = declare_parameter<bool>("print_enable", true);
    print_throttle_ms_ = declare_parameter<int>("print_throttle_ms", 500);

    start_time_ = now();

    // ---------------- State init ----------------
    const std::size_t M = odom_topics_.size();
    drones_.resize(M);
    have_pose_.assign(M, false);

    pub_assign_.resize(M);
    pub_assigned_obstacle_.resize(M);
    pub_assigned_pose_.resize(M);

    pub_mode_.resize(M);
    pub_done_.resize(M);
    pub_done_target_.resize(M);
    pub_qstar_dxdy_.resize(M);

    // Stable “stickiness memory” by track_id (not index)
    last_primary_track_id_.assign(M, -1);

    assignments_per_drone_.resize(M);
    drone_ns_.resize(M);
    last_done_target_id_.assign(M, -1);

    // q* storage
    last_qstar_pose_.resize(M);
    last_qstar_valid_.assign(M, false);
    for (size_t j = 0; j < M; ++j) last_qstar_pose_[j].orientation.w = 1.0;

    // metrics
    last_sep_penalty_.assign(M, 0.0);

    // Latch SURVEILLANCE per drone (stays true after DONE until reset)
    latched_surveillance_.assign(M, false);

    // per-drone print throttle state
    last_print_time_.assign(M, rclcpp::Time(0, 0, get_clock()->get_clock_type()));

    if (publish_metrics_) {
      metrics_pub_ = create_publisher<msgs::AssignmentMetrics>(metrics_topic_, 10);
      RCLCPP_INFO(get_logger(), "AssignmentMetrics publishing on '%s'", metrics_topic_.c_str());
    }

    // ---------------- Subscribers / Publishers ----------------
    for (size_t i = 0; i < M; ++i) {
      const auto & odom_topic = odom_topics_[i];

      auto sub = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, QoS(20),
        [this, i](const nav_msgs::msg::Odometry::SharedPtr msg)
        {
          drones_[i].position = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
          );

          // yaw from orientation
          const auto &q = msg->pose.pose.orientation;
          double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
          double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
          drones_[i].yaw = std::atan2(siny_cosp, cosy_cosp);

          have_pose_[i] = true;
        });
      odom_subs_.push_back(sub);

      auto ns = ns_from_topic(odom_topic);
      drone_ns_[i] = ns;

      const std::string mode_topic    = ns.empty()? "mode"            : ns + "/mode";
      const std::string done_topic    = ns.empty()? "tracking_done"   : ns + "/tracking_done";
      const std::string done_id_topic = ns.empty()? "done_target_id"  : ns + "/done_target_id";
      const std::string qdxdy_topic   = ns.empty()? "qstar_dxdy"      : ns + "/qstar_dxdy";

      pub_mode_[i]        = create_publisher<std_msgs::msg::String>(mode_topic, 10);
      pub_done_[i]        = create_publisher<std_msgs::msg::Bool>(done_topic, 10);
      pub_done_target_[i] = create_publisher<std_msgs::msg::Int32>(done_id_topic, 10);
      pub_qstar_dxdy_[i]  = create_publisher<geometry_msgs::msg::Vector3Stamped>(qdxdy_topic, 10);

      const std::string assign_topic    = ns.empty()? "assign_target"       : ns + "/assign_target";
      const std::string assigned_obs_t  = ns.empty()? "assigned_obstacle"   : ns + "/assigned_obstacle";
      const std::string assigned_pose_t = ns.empty()? "assigned_pose"       : ns + "/assigned_pose";

      pub_assign_[i]            = create_publisher<msgs::AssignTarget>(assign_topic, 10);
      pub_assigned_obstacle_[i] = create_publisher<msgs::TrackedObstacle>(assigned_obs_t, 10);
      pub_assigned_pose_[i]     = create_publisher<geometry_msgs::msg::PoseStamped>(assigned_pose_t, 10);

      // Per-drone reset topic: /droneX/reset_tracking (std_msgs/Bool true)
      {
        const std::string reset_t = ns.empty() ? "reset_tracking" : (ns + "/reset_tracking");
        auto rsub = create_subscription<std_msgs::msg::Bool>(
          reset_t, QoS(10),
          [this, i, reset_t](const std_msgs::msg::Bool::SharedPtr msg)
          {
            if (!msg->data) return;
            latched_surveillance_[i] = false;
            last_done_target_id_[i]  = -1;
            last_primary_track_id_[i] = -1;

            // NOTE:
            // We do NOT clear done_class_ids_ here, because your request is global lockout:
            // once any class is DONE, it stays blocked for all drones until global reset.
            RCLCPP_WARN(get_logger(), "[RESET] %s unlocked (topic=%s) -> TRACKING allowed (global DONE classes stay locked)",
                        drone_ns_[i].c_str(), reset_t.c_str());
          });
        reset_subs_.push_back(rsub);
      }

      RCLCPP_INFO(get_logger(),
        "Odom[%zu] '%s' -> pubs: '%s', '%s', '%s'",
        i, odom_topic.c_str(), assign_topic.c_str(), assigned_obs_t.c_str(), assigned_pose_t.c_str());
    }

    // Global reset topic: /reset_tracking (std_msgs/Bool true) unlocks ALL drones AND clears DONE classes
    reset_all_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/reset_tracking", QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        if (!msg->data) return;
        for (size_t i = 0; i < latched_surveillance_.size(); ++i) {
          latched_surveillance_[i] = false;
          last_done_target_id_[i]  = -1;
          last_primary_track_id_[i] = -1;
        }
        done_class_ids_.clear();
        RCLCPP_WARN(get_logger(), "[RESET] ALL drones unlocked via /reset_tracking; DONE classes cleared");
      });

    // Fused tracks subscriber
    tracks_sub_ = create_subscription<msgs::TrackedObstacleArray>(
      "fused_tracked_obstacles_array", QoS(10),
      std::bind(&AssignmentNode::tracksCallback, this, std::placeholders::_1));

    // GT from container odometry topics (association by class_id)
    setupGtOdomSubscribers();

    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("assignment_markers", 10);

    // CSV open
    openCsvIfEnabled();

    auto period = std::chrono::duration<double>(1.0 / std::max(alloc_rate_hz_, 1e-3));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AssignmentNode::tick, this));

    RCLCPP_INFO(get_logger(),
      "assignment_node up. drones=%zu alloc=%.2f Hz | K=%d | hover: L=%d r=%.1f h=%.1f fixed_alt=%s(%.1f)",
      odom_topics_.size(), alloc_rate_hz_, drone_capacity_,
      hover_params_.l, hover_params_.r_h, hover_params_.h,
      use_fixed_altitude_ ? "true":"false", fixed_altitude_m_);

    RCLCPP_INFO(get_logger(),
      "Stabilization: stickiness based on track_id; switch_penalty=%.3f",
      switch_penalty_);

    RCLCPP_INFO(get_logger(),
      "DONE policy: GLOBAL lockout by class_id (container identity). Once DONE, class is blocked for ALL drones until /reset_tracking.");

    RCLCPP_INFO(get_logger(),
      "Mode logic: start SURVEILLANCE when no targets; switch to TRACKING when detections exist; "
      "switch back to SURVEILLANCE when DONE; latch until reset."
      " Per-drone: /droneX/reset_tracking, Global: /reset_tracking (std_msgs/Bool true)");

    RCLCPP_INFO(get_logger(),
      "Printing: print_enable=%s, print_throttle_ms=%d (per-drone)",
      print_enable_ ? "true":"false", print_throttle_ms_);
  }

  ~AssignmentNode() override
  {
    if (csv_.is_open()) csv_.close();
  }

private:
  // ---------- ROS ----------
  rclcpp::Subscription<msgs::TrackedObstacleArray>::SharedPtr tracks_sub_;

  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  std::vector<rclcpp::Publisher<msgs::AssignTarget>::SharedPtr> pub_assign_;
  std::vector<rclcpp::Publisher<msgs::TrackedObstacle>::SharedPtr> pub_assigned_obstacle_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pub_assigned_pose_;
  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pub_mode_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> pub_done_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> pub_done_target_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr> pub_qstar_dxdy_;
  rclcpp::TimerBase::SharedPtr timer_;
  std_msgs::msg::Header last_header_;

  // reset subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_all_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> reset_subs_;

  // metrics
  bool publish_metrics_{true};
  std::string metrics_topic_{"assignment_metrics"};
  rclcpp::Publisher<msgs::AssignmentMetrics>::SharedPtr metrics_pub_;

  // ---------- CSV ----------
  bool csv_enable_{false};
  bool csv_append_{false};
  bool csv_flush_each_tick_{true};
  std::string csv_path_;
  std::ofstream csv_;
  uint64_t csv_seq_{0};

  // ---------- Params/state ----------
  double alloc_rate_hz_{1.0};
  double eta_{1.0}, beta_{0.2}, gamma_{0.3}, rho_{0.2}, kappa_{1000.0};
  double switch_penalty_{0.0}; // NEW

  double r_safe_{0.5}, d_max_{25.0};
  int    drone_capacity_{1};

  double min_alt_{1.0}, max_alt_{50.0};
  bool   verbose_{false};

  // termination
  double tau_logdet_{-5.0};
  double tau_dJ_{1e-3};
  double tmax_meas_{2.0};
  bool   print_term_{false};

  bool require_reach_and_term_{true};

  // fixed altitude flight
  bool use_fixed_altitude_{true};
  double fixed_altitude_m_{10.0};

  // visualization toggles
  bool viz_ring_{true};
  bool viz_qstar_{true};

  bool publish_qstar_dxdy_{true};

  HoverParams hover_params_;

  std::vector<DroneState>   drones_;
  std::vector<bool>         have_pose_;
  std::vector<TargetTrack>  tracks_;

  // stickiness memory by track_id (not index)
  std::vector<int>          last_primary_track_id_;

  std::vector<std::string>  drone_ns_;
  std::vector<std::string>  odom_topics_;
  std::vector<std::vector<int>> assignments_per_drone_;

  Eigen::MatrixXd tilde_deltaJ_;
  Eigen::MatrixXd last_dist_;
  std::vector<double> last_sep_penalty_;

  // per-target staleness timestamps (id -> time)
  std::unordered_map<int, rclcpp::Time> last_seen_by_id_;

  // NEW: containers declared DONE globally by class_id (container identity)
  std::unordered_set<int> done_class_ids_;

  // per-drone done bookkeeping (stores last DONE track id for that drone; useful for UI)
  std::vector<int> last_done_target_id_;

  // q* storage
  std::vector<geometry_msgs::msg::Pose> last_qstar_pose_;
  std::vector<bool> last_qstar_valid_;

  // time
  rclcpp::Time start_time_;

  // INSIDE circle + best-estimator
  double hover_inside_tol_m_{0.3};
  double hover_z_tol_m_{0.5};
  bool   require_best_estimator_{true};
  double best_margin_logdet_{0.05};

  // latch drone to SURVEILLANCE until reset
  std::vector<bool> latched_surveillance_;

  // -------- GT cache from /containerX/odometry (keyed by container_id 1..N) --------
  bool gt_enable_{false};
  double gt_max_age_s_{1.0};
  std::vector<std::string> gt_odom_topics_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> gt_odom_subs_;

  std::unordered_map<int, Eigen::Vector3d> gt_pos_by_container_id_;
  std::unordered_map<int, rclcpp::Time>    gt_stamp_by_container_id_;

  // -------- PER-DRONE PRINTING --------
  bool print_enable_{true};
  int  print_throttle_ms_{500};
  std::vector<rclcpp::Time> last_print_time_;

  // ---------- callbacks ----------
  void tracksCallback(const msgs::TrackedObstacleArray::SharedPtr msg)
  {
    last_header_ = msg->header;
    tracks_.clear();
    tracks_.reserve(msg->obstacles.size());

    rclcpp::Time stamp = rclcpp::Time(msg->header.stamp);
    if (stamp.nanoseconds() == 0) stamp = now();

    for (const auto & o : msg->obstacles) {
      TargetTrack t;
      t.mean = Eigen::Vector3d(o.position.x, o.position.y, o.position.z);

      Eigen::Matrix3d P;
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          P(r,c) = o.covariance[r*3 + c];
      t.covariance = P;

      t.class_id   = o.class_id;
      t.id         = o.id;

      last_seen_by_id_[t.id] = stamp;
      tracks_.push_back(t);
    }
  }

  // ---------- setup GT odom subscribers ----------
  void setupGtOdomSubscribers()
  {
    if (!gt_enable_) {
      RCLCPP_INFO(get_logger(), "GT disabled (gt_enable=false). gt=NO(...) expected.");
      return;
    }

    // Extract container number from "/containerX/odometry" -> X (int)
    auto parse_container_id = [](const std::string& topic) -> int {
      std::smatch m;
      static const std::regex re("^/container([0-9]+)/odometry$");
      if (std::regex_match(topic, m, re)) return std::stoi(m[1].str());
      return -1;
    };

    gt_odom_subs_.reserve(gt_odom_topics_.size());

    size_t ok = 0, bad = 0;
    for (const auto& topic : gt_odom_topics_) {
      const int container_id = parse_container_id(topic);
      if (container_id < 0) {
        RCLCPP_WARN(get_logger(),
                    "GT topic '%s' does not match '/containerX/odometry' -> skipped",
                    topic.c_str());
        bad++;
        continue;
      }

      auto sub = create_subscription<nav_msgs::msg::Odometry>(
        topic, QoS(10),
        [this, container_id](const nav_msgs::msg::Odometry::SharedPtr msg)
        {
          Eigen::Vector3d p(msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            msg->pose.pose.position.z);

          rclcpp::Time st(msg->header.stamp);
          if (st.nanoseconds() == 0) st = now();

          gt_pos_by_container_id_[container_id]   = p;
          gt_stamp_by_container_id_[container_id] = st;
        });

      gt_odom_subs_.push_back(sub);
      ok++;

      RCLCPP_INFO(get_logger(),
                  "GT subscribed: container_id=%d topic='%s'",
                  container_id, topic.c_str());
    }

    RCLCPP_INFO(get_logger(),
                "GT association ENABLED (class_id 0..4 -> container_id 1..5). subscribers=%zu (skipped=%zu). max_age=%.2fs",
                ok, bad, gt_max_age_s_);
  }

  // ---------- GT helper (by container_id 1..N) ----------
  bool getFreshGTByContainerId(int container_id, Eigen::Vector3d& gt_out, double& age_s_out) const
  {
    if (!gt_enable_) return false;
    if (container_id < 1) return false;

    auto itp = gt_pos_by_container_id_.find(container_id);
    auto its = gt_stamp_by_container_id_.find(container_id);
    if (itp == gt_pos_by_container_id_.end() || its == gt_stamp_by_container_id_.end()) return false;

    const double age = (now() - its->second).seconds();
    age_s_out = age;
    if (age > gt_max_age_s_) return false;

    gt_out = itp->second;
    return true;
  }

  // ---------- math helpers ----------
  double logdetP(const Eigen::Matrix3d& P) const {
    return std::log(std::max(1e-12, P.determinant()));
  }

  bool isTargetStale(int target_id) const {
    auto it = last_seen_by_id_.find(target_id);
    if (it == last_seen_by_id_.end()) return true;
    return (now() - it->second).seconds() > tmax_meas_;
  }

  double dJ_at_pose(const TargetTrack& trk, const Eigen::Vector3d& sensor_pos_world) const
  {
    const Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R = rangeAwareR(sensor_pos_world, trk.mean, hover_params_.meas_sigma0, hover_params_.meas_k_range);
    return dOptimalityGain(trk.covariance, H, R);
  }

  bool terminationSatisfiedForDrone(size_t drone_j, size_t track_index) const
  {
    if (track_index >= tracks_.size()) return false;
    const auto& trk = tracks_[track_index];

    const bool cond_logdet = (logdetP(trk.covariance) <= tau_logdet_);
    const bool cond_stale  = isTargetStale(trk.id);

    Eigen::Vector3d sensor = drones_[drone_j].position;
    if (use_fixed_altitude_) sensor.z() = fixed_altitude_m_;
    const double dJ = dJ_at_pose(trk, sensor);
    const bool cond_dj = (dJ <= tau_dJ_);

    return cond_logdet || cond_dj || cond_stale;
  }

  bool reachedHoverCircle(size_t drone_j, size_t track_index) const
  {
    if (track_index >= tracks_.size()) return false;

    const Eigen::Vector3d pj = drones_[drone_j].position;
    const Eigen::Vector3d pi = tracks_[track_index].mean;

    const double ring_z = use_fixed_altitude_ ? fixed_altitude_m_
                                             : (pi.z() + hover_params_.h);

    const double dx = pj.x() - pi.x();
    const double dy = pj.y() - pi.y();
    const double r_xy = std::sqrt(dx*dx + dy*dy);

    const bool inside_xy = (r_xy <= (hover_params_.r_h + hover_inside_tol_m_));
    const bool ok_z      = (std::fabs(pj.z() - ring_z) <= hover_z_tol_m_);

    return inside_xy && ok_z;
  }

  double predictedPosteriorLogdet(size_t drone_j, size_t track_index) const
  {
    const auto& trk = tracks_[track_index];
    Eigen::Vector3d sensor = drones_[drone_j].position;
    if (use_fixed_altitude_) sensor.z() = fixed_altitude_m_;

    Eigen::Matrix3d R = rangeAwareR(sensor, trk.mean,
                                    hover_params_.meas_sigma0,
                                    hover_params_.meas_k_range);

    const Eigen::Matrix3d Pinv = trk.covariance.inverse();
    const Eigen::Matrix3d Pp   = (Pinv + R.inverse()).inverse(); // H=I
    return logdetP(Pp);
  }

  bool isBestEstimatorForTarget(size_t drone_j, size_t track_index) const
  {
    const double mine = predictedPosteriorLogdet(drone_j, track_index);

    double best = std::numeric_limits<double>::infinity();
    for (size_t k = 0; k < drones_.size(); ++k)
      best = std::min(best, predictedPosteriorLogdet(k, track_index));

    return (mine <= best + best_margin_logdet_);
  }

  // ---------- PER-DRONE TERMINAL PRINTING ----------
  void printImportantPerDrone(const std::vector<int>& primary)
  {
    if (!print_enable_) return;

    const size_t M = drones_.size();
    const size_t N = tracks_.size();
    const auto now_t = now();

    const double throttle_s = std::max(0.0, (double)print_throttle_ms_ / 1000.0);

    for (size_t j = 0; j < M; ++j) {
      if (throttle_s > 0.0) {
        const double dt = (now_t - last_print_time_[j]).seconds();
        if (dt < throttle_s) continue;
      }
      last_print_time_[j] = now_t;

      const bool latched = latched_surveillance_[j];

      std::string mode = "SURVEILLANCE";
      if (N > 0 && !latched) {
        mode = (primary.size() > j && primary[j] >= 0) ? "TRACKING" : "SURVEILLANCE";
      }

      int primary_track_idx = (primary.size() > j) ? primary[j] : -1;

      int track_id  = -1;
      int class_id  = -1;

      Eigen::Vector3d est = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

      int has_gt = 0;
      double gt_age_s = std::numeric_limits<double>::quiet_NaN();
      Eigen::Vector3d gt = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

      int at_circle = 0;
      int term = 0;
      int cond_logdet = 0;
      int cond_dj = 0;
      int cond_stale = 0;

      if (primary_track_idx >= 0 && primary_track_idx < (int)tracks_.size()) {
        const auto& trk = tracks_[(size_t)primary_track_idx];
        track_id = trk.id;
        class_id = trk.class_id;
        est      = trk.mean;

        at_circle = reachedHoverCircle(j, (size_t)primary_track_idx) ? 1 : 0;

        const double tlogdet = logdetP(trk.covariance);
        cond_logdet = (tlogdet <= tau_logdet_) ? 1 : 0;
        cond_stale  = isTargetStale(trk.id) ? 1 : 0;

        Eigen::Vector3d sensor = drones_[j].position;
        if (use_fixed_altitude_) sensor.z() = fixed_altitude_m_;
        const double dJ_cur = dJ_at_pose(trk, sensor);
        cond_dj = (dJ_cur <= tau_dJ_) ? 1 : 0;

        term = (cond_logdet || cond_dj || cond_stale) ? 1 : 0;

        // GT by class_id (0..4) -> container_id (1..5)
        const int container_id = container_id_from_class_id(class_id);
        Eigen::Vector3d gt_tmp;
        double age_s = 0.0;
        if (getFreshGTByContainerId(container_id, gt_tmp, age_s)) {
          has_gt = 1;
          gt_age_s = age_s;
          gt = gt_tmp;
        }
      }

      const bool class_done = (class_id >= 0 && done_class_ids_.count(class_id) > 0);

      RCLCPP_INFO(
        get_logger(),
        "[ASSIGN] drone=%zu ns=%s mode=%s latched=%d | track_id=%d class_id=%d done_class=%d | "
        "est=(%.2f,%.2f,%.2f) gt=%s(%.2f,%.2f,%.2f) age=%.2f | "
        "at_circle=%d term=%d [logdet=%d dj=%d stale=%d]",
        j, drone_ns_[j].c_str(), mode.c_str(), (int)latched,
        track_id, class_id, (int)class_done,
        est.x(), est.y(), est.z(),
        (has_gt ? "YES" : "NO"),
        gt.x(), gt.y(), gt.z(), gt_age_s,
        at_circle, term,
        cond_logdet, cond_dj, cond_stale
      );
    }
  }

  // ---------- cost model ----------
  Eigen::MatrixXd buildCostMatrix()
  {
    const size_t M = drones_.size();
    const size_t N = tracks_.size();

    Eigen::MatrixXd C(M, N);
    C.setZero();

    tilde_deltaJ_.resize(M, N);
    last_dist_.resize(M, N);

    // Separation penalty φ_j
    std::vector<double> sep_penalty(M, 0.0);
    for (size_t j = 0; j < M; ++j) {
      double min_sep = std::numeric_limits<double>::infinity();
      for (size_t k = 0; k < M; ++k) {
        if (k == j) continue;
        double d = (drones_[j].position - drones_[k].position).norm();
        min_sep = std::min(min_sep, d);
      }
      sep_penalty[j] = std::isfinite(min_sep) ? std::max(0.0, r_safe_ - min_sep) : 0.0;
    }
    last_sep_penalty_ = sep_penalty;

    const Eigen::Matrix3d H = Eigen::Matrix3d::Identity();

    for (size_t j = 0; j < M; ++j) {
      const Eigen::Vector3d &r_j = drones_[j].position;

      for (size_t i = 0; i < N; ++i) {
        const auto &trk = tracks_[i];
        const Eigen::Vector3d &p_i = trk.mean;

        const double dist = (p_i - r_j).norm();
        last_dist_((int)j, (int)i) = dist;

        // BLOCK: class already DONE (global lockout by class_id)
        if (trk.class_id >= 0 && done_class_ids_.count(trk.class_id) > 0) {
          tilde_deltaJ_(j,i) = 0.0;
          C((int)j,(int)i) = kappa_;
          continue;
        }

        const Eigen::Matrix3d R_ji = rangeAwareR(
          r_j, p_i, hover_params_.meas_sigma0, hover_params_.meas_k_range
        );

        const double dJ_ji = dOptimalityGain(trk.covariance, H, R_ji);
        tilde_deltaJ_(j, i) = dJ_ji;

        const bool infeasible = (dist > d_max_);
        const double infeas_cost = infeasible ? kappa_ : 0.0;

        const double cost_info  = -eta_ * dJ_ji;
        const double cost_dist  =  beta_ * dist;

        // stickiness based on stable track_id (not index)
        const double stick = (last_primary_track_id_[j] != -1 && last_primary_track_id_[j] == trk.id) ? 1.0 : 0.0;
        const double cost_stick = -rho_ * stick;

        // explicit switch penalty when changing away from last primary
        double cost_switch = 0.0;
        if (switch_penalty_ > 0.0 && last_primary_track_id_[j] != -1 && trk.id != last_primary_track_id_[j]) {
          cost_switch = switch_penalty_;
        }

        const double cost_sep   =  gamma_ * sep_penalty[j];

        C((int)j, (int)i) = cost_info + cost_dist + cost_stick + cost_switch + cost_sep + infeas_cost;
      }
    }

    return C;
  }

  // ---------- hover selection (q*) ----------
  geometry_msgs::msg::Pose chooseBestHoverPose(
    size_t j,
    const Eigen::Vector3d& s_j,
    const TargetTrack& trk)
  {
    std::vector<Eigen::Vector3d> others; others.reserve(drones_.size()-1);
    for (size_t k=0; k<drones_.size(); ++k) if (k!=j) others.push_back(drones_[k].position);

    const Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d p_i = trk.mean;

    double best = -1e18;
    Eigen::Vector3d best_q = p_i + Eigen::Vector3d(hover_params_.r_h, 0.0, hover_params_.h);

    const double ring_z = use_fixed_altitude_ ? fixed_altitude_m_ : (p_i.z() + hover_params_.h);

    for (int ell=0; ell<hover_params_.l; ++ell) {
      const double psi = 2.0 * kPi * (double)ell / (double)hover_params_.l;

      Eigen::Vector3d q(
        p_i.x() + hover_params_.r_h*std::cos(psi),
        p_i.y() + hover_params_.r_h*std::sin(psi),
        ring_z
      );

      if (!altitudeOK(q, min_alt_, max_alt_)) continue;
      if (!geofenceOK(q)) continue;
      if (!losOK(q, p_i)) continue;
      if (minSepAt(q, others) < hover_params_.r_safe) continue;

      const double tsec = travelSeconds(s_j, q, hover_params_.v_travel);
      if (tsec > hover_params_.tmax) continue;
      if (!batteryOK()) continue;

      const Eigen::Matrix3d R = rangeAwareR(q, p_i, hover_params_.meas_sigma0, hover_params_.meas_k_range);
      const double dJ = dOptimalityGain(trk.covariance, H, R);
      const double score = dJ / (tsec + hover_params_.eps_time);

      if (score > best) { best = score; best_q = q; }
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = best_q.x();
    pose.position.y = best_q.y();
    pose.position.z = best_q.z();
    pose.orientation.w = 1.0;
    return pose;
  }

  // ---------- visualization helpers ----------
  visualization_msgs::msg::Marker makeRingMarkerWorld(
    int marker_id,
    const std::string& frame_id,
    const Eigen::Vector3d& center_world,
    double ring_z_world,
    double r_h,
    int L)
  {
    visualization_msgs::msg::Marker mk;
    mk.header.stamp = now();
    mk.header.frame_id = frame_id;
    mk.ns = "qstar_ring";
    mk.id = marker_id;
    mk.type = visualization_msgs::msg::Marker::LINE_STRIP;
    mk.action = visualization_msgs::msg::Marker::ADD;

    mk.scale.x = 0.03;
    mk.color.a = 0.9;
    mk.color.r = 0.0;
    mk.color.g = 1.0;
    mk.color.b = 1.0;

    mk.pose.orientation.w = 1.0;

    mk.points.reserve(L + 1);
    for (int ell = 0; ell <= L; ++ell) {
      double psi = 2.0 * kPi * (double)ell / (double)L;
      geometry_msgs::msg::Point p;
      p.x = center_world.x() + r_h * std::cos(psi);
      p.y = center_world.y() + r_h * std::sin(psi);
      p.z = ring_z_world;
      mk.points.push_back(p);
    }

    mk.lifetime = rclcpp::Duration::from_seconds(0.7);
    return mk;
  }

  visualization_msgs::msg::Marker makeQstarMarkerWorld(
    int marker_id,
    const std::string& frame_id,
    const geometry_msgs::msg::Pose& qstar_world)
  {
    visualization_msgs::msg::Marker mk;
    mk.header.stamp = now();
    mk.header.frame_id = frame_id;
    mk.ns = "qstar_point";
    mk.id = marker_id;
    mk.type = visualization_msgs::msg::Marker::SPHERE;
    mk.action = visualization_msgs::msg::Marker::ADD;

    mk.pose = qstar_world;
    mk.scale.x = 0.18;
    mk.scale.y = 0.18;
    mk.scale.z = 0.18;

    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 0.2;
    mk.color.b = 0.2;

    mk.lifetime = rclcpp::Duration::from_seconds(0.7);
    return mk;
  }

  // ---------- CSV ----------
  void openCsvIfEnabled()
  {
    if (!csv_enable_) return;
    if (csv_path_.empty()) {
      RCLCPP_WARN(get_logger(), "csv_enable=true but csv_path is empty");
      return;
    }

    try {
      std::filesystem::create_directories(std::filesystem::path(csv_path_).parent_path());
      const auto mode = csv_append_ ? std::ios::app : std::ios::trunc;
      csv_.open(csv_path_, mode);
      if (!csv_.is_open()) {
        RCLCPP_ERROR(get_logger(), "CSV enabled but failed to open: %s", csv_path_.c_str());
        return;
      }

      bool write_header = !csv_append_;
      if (csv_append_) {
        try {
          if (std::filesystem::exists(csv_path_) && std::filesystem::file_size(csv_path_) == 0) {
            write_header = true;
          }
        } catch (...) {}
      }

      if (write_header) {
        csv_ <<
          "seq,t_sec,stamp_sec,stamp_nsec,"
          "drone_idx,drone_ns,mode,latched_surveillance,last_done_target_id,"
          "num_targets,primary_track_idx,primary_track_id,primary_class_id,"
          "drone_x,drone_y,drone_z,drone_yaw,"
          "target_x,target_y,target_z,target_logdetP,"
          "qstar_valid,qstar_x,qstar_y,qstar_z,"
          "dopt_gain,dist,cost,sep_penalty,"
          "ring_z,r_xy,z_err,at_circle,"
          "term,cond_logdet,cond_dj,cond_stale,dJ_cur,post_logdetP,"
          "has_gt,gt_age_s,gt_x,gt_y,gt_z,err_x,err_y,err_z,err_norm\n";
        csv_.flush();
      }

      RCLCPP_INFO(get_logger(), "CSV logging -> %s (append=%d)", csv_path_.c_str(), (int)csv_append_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "CSV setup exception: %s", e.what());
    }
  }

  void logCsvPerDrone(const std::vector<int>& primary, const Eigen::MatrixXd* Cptr)
  {
    if (!csv_enable_ || !csv_.is_open()) return;

    const double t_sec = (now() - start_time_).seconds();
    const auto stamp = now();

    const int64_t stamp_sec  = static_cast<int64_t>(stamp.seconds());
    const uint32_t stamp_nsec = static_cast<uint32_t>((stamp.nanoseconds()) % 1000000000LL);

    const size_t M = drones_.size();
    const size_t N = tracks_.size();

    for (size_t j = 0; j < M; ++j) {
      const bool latched = latched_surveillance_[j];

      std::string mode = "SURVEILLANCE";
      if (N > 0 && !latched) {
        mode = (primary.size() > j && primary[j] >= 0) ? "TRACKING" : "SURVEILLANCE";
      }

      int primary_track_idx = (primary.size() > j) ? primary[j] : -1;
      int primary_track_id  = -1;
      int primary_class_id  = -1;

      const auto& dp = drones_[j].position;
      const double yaw = drones_[j].yaw;

      double tx = std::numeric_limits<double>::quiet_NaN();
      double ty = std::numeric_limits<double>::quiet_NaN();
      double tz = std::numeric_limits<double>::quiet_NaN();
      double tlogdet = std::numeric_limits<double>::quiet_NaN();

      double dopt = std::numeric_limits<double>::quiet_NaN();
      double dist = std::numeric_limits<double>::quiet_NaN();
      double cost = std::numeric_limits<double>::quiet_NaN();
      double sep_pen = (j < last_sep_penalty_.size()) ? last_sep_penalty_[j] : 0.0;

      double ring_z = std::numeric_limits<double>::quiet_NaN();
      double r_xy   = std::numeric_limits<double>::quiet_NaN();
      double z_err  = std::numeric_limits<double>::quiet_NaN();
      int at_circle = 0;

      int term = 0;
      int cond_logdet = 0;
      int cond_dj = 0;
      int cond_stale = 0;
      double dJ_cur = std::numeric_limits<double>::quiet_NaN();
      double post_logdet = std::numeric_limits<double>::quiet_NaN();

      int has_gt = 0;
      double gt_age_s = std::numeric_limits<double>::quiet_NaN();
      double gtx = std::numeric_limits<double>::quiet_NaN();
      double gty = std::numeric_limits<double>::quiet_NaN();
      double gtz = std::numeric_limits<double>::quiet_NaN();
      double errx = std::numeric_limits<double>::quiet_NaN();
      double erry = std::numeric_limits<double>::quiet_NaN();
      double errz = std::numeric_limits<double>::quiet_NaN();
      double errn = std::numeric_limits<double>::quiet_NaN();

      if (primary_track_idx >= 0 && primary_track_idx < (int)N) {
        const auto& trk = tracks_[(size_t)primary_track_idx];
        primary_track_id = trk.id;
        primary_class_id = trk.class_id;

        tx = trk.mean.x(); ty = trk.mean.y(); tz = trk.mean.z();
        tlogdet = logdetP(trk.covariance);

        if (tilde_deltaJ_.rows() == (int)M && tilde_deltaJ_.cols() == (int)N) dopt = tilde_deltaJ_((int)j, primary_track_idx);
        if (last_dist_.rows() == (int)M && last_dist_.cols() == (int)N)       dist = last_dist_((int)j, primary_track_idx);
        if (Cptr && Cptr->rows() == (int)M && Cptr->cols() == (int)N)         cost = (*Cptr)((int)j, primary_track_idx);

        ring_z = use_fixed_altitude_ ? fixed_altitude_m_ : (trk.mean.z() + hover_params_.h);

        const double dx = dp.x() - trk.mean.x();
        const double dy = dp.y() - trk.mean.y();
        r_xy = std::sqrt(dx*dx + dy*dy);
        z_err = std::fabs(dp.z() - ring_z);

        at_circle = reachedHoverCircle(j, (size_t)primary_track_idx) ? 1 : 0;

        cond_logdet = (tlogdet <= tau_logdet_) ? 1 : 0;
        cond_stale  = isTargetStale(trk.id) ? 1 : 0;

        Eigen::Vector3d sensor = dp;
        if (use_fixed_altitude_) sensor.z() = fixed_altitude_m_;
        dJ_cur = dJ_at_pose(trk, sensor);
        cond_dj = (dJ_cur <= tau_dJ_) ? 1 : 0;

        term = (cond_logdet || cond_dj || cond_stale) ? 1 : 0;

        post_logdet = predictedPosteriorLogdet(j, (size_t)primary_track_idx);

        // GT by class_id (0..4) -> container_id (1..5)
        const int container_id = container_id_from_class_id(primary_class_id);
        Eigen::Vector3d gt;
        double age_s = 0.0;
        if (getFreshGTByContainerId(container_id, gt, age_s)) {
          has_gt = 1;
          gt_age_s = age_s;
          gtx = gt.x(); gty = gt.y(); gtz = gt.z();

          Eigen::Vector3d est(tx, ty, tz);
          Eigen::Vector3d e = est - gt;
          errx = e.x(); erry = e.y(); errz = e.z();
          errn = e.norm();
        }
      }

      int qvalid = last_qstar_valid_[j] ? 1 : 0;
      double qx = std::numeric_limits<double>::quiet_NaN();
      double qy = std::numeric_limits<double>::quiet_NaN();
      double qz = std::numeric_limits<double>::quiet_NaN();
      if (last_qstar_valid_[j]) {
        qx = last_qstar_pose_[j].position.x;
        qy = last_qstar_pose_[j].position.y;
        qz = last_qstar_pose_[j].position.z;
      }

      csv_ << csv_seq_++ << ","
           << std::fixed << std::setprecision(3)
           << t_sec << ","
           << stamp_sec << "," << stamp_nsec << ","
           << j << ","
           << drone_ns_[j] << ","
           << mode << ","
           << (latched ? 1 : 0) << ","
           << last_done_target_id_[j] << ","
           << N << ","
           << primary_track_idx << ","
           << primary_track_id << ","
           << primary_class_id << ","
           << dp.x() << "," << dp.y() << "," << dp.z() << ","
           << yaw << ","
           << tx << "," << ty << "," << tz << "," << tlogdet << ","
           << qvalid << "," << qx << "," << qy << "," << qz << ","
           << dopt << "," << dist << "," << cost << "," << sep_pen << ","
           << ring_z << "," << r_xy << "," << z_err << "," << at_circle << ","
           << term << "," << cond_logdet << "," << cond_dj << "," << cond_stale << ","
           << dJ_cur << "," << post_logdet << ","
           << has_gt << "," << gt_age_s << ","
           << gtx << "," << gty << "," << gtz << ","
           << errx << "," << erry << "," << errz << "," << errn
           << "\n";
    }

    if (csv_flush_each_tick_) csv_.flush();
  }

  // ---------- publish metrics ----------
  void publishMetrics(const std::vector<int>& primary, const Eigen::MatrixXd& C)
  {
    if (!publish_metrics_ || !metrics_pub_) return;

    const size_t M = drones_.size();
    const size_t N = tracks_.size();
    if (M == 0 || N == 0) return;

    msgs::AssignmentMetrics m;
    m.header = last_header_;
    m.header.stamp = now();
    if (m.header.frame_id.empty()) m.header.frame_id = "world";

    m.num_drones  = static_cast<uint32_t>(M);
    m.num_targets = static_cast<uint32_t>(N);

    m.drone_ns = drone_ns_;
    m.target_ids.resize(N);
    m.target_class_ids.resize(N);

    m.drone_pos_world.resize(M);
    m.target_pos_world.resize(N);
    m.target_logdet_p.resize(N);

    for (size_t j=0; j<M; ++j) {
      geometry_msgs::msg::Point p;
      p.x = drones_[j].position.x();
      p.y = drones_[j].position.y();
      p.z = drones_[j].position.z();
      m.drone_pos_world[j] = p;
    }

    for (size_t i=0; i<N; ++i) {
      m.target_ids[i] = tracks_[i].id;
      m.target_class_ids[i] = tracks_[i].class_id;

      geometry_msgs::msg::Point p;
      p.x = tracks_[i].mean.x();
      p.y = tracks_[i].mean.y();
      p.z = tracks_[i].mean.z();
      m.target_pos_world[i] = p;

      m.target_logdet_p[i] = logdetP(tracks_[i].covariance);
    }

    m.dopt.resize(M*N);
    m.dist.resize(M*N);
    m.cost.resize(M*N);

    for (size_t j=0; j<M; ++j) {
      for (size_t i=0; i<N; ++i) {
        size_t k = j*N + i;
        m.dopt[k] = tilde_deltaJ_((int)j, (int)i);
        m.dist[k] = last_dist_((int)j, (int)i);
        m.cost[k] = C((int)j, (int)i);
      }
    }

    m.sep_penalty.resize(M);
    for (size_t j=0; j<M; ++j) m.sep_penalty[j] = last_sep_penalty_[j];

    m.primary_track_index.resize(M);
    m.primary_target_id.resize(M);
    for (size_t j=0; j<M; ++j) {
      m.primary_track_index[j] = primary[j];
      m.primary_target_id[j] = (primary[j]>=0 && primary[j]<(int)N) ? tracks_[primary[j]].id : -1;
    }

    m.qstar_world = last_qstar_pose_;
    m.has_qstar   = last_qstar_valid_;

    // params (optional)
    m.eta = eta_; m.beta = beta_; m.gamma = gamma_; m.rho = rho_; m.kappa = kappa_;
    m.d_max = d_max_; m.r_safe = r_safe_;
    m.hover_l = static_cast<uint32_t>(hover_params_.l);
    m.hover_radius = hover_params_.r_h;
    m.hover_height = hover_params_.h;
    m.travel_speed = hover_params_.v_travel;
    m.hover_tmax = hover_params_.tmax;
    m.meas_sigma0 = hover_params_.meas_sigma0;
    m.meas_k_range = hover_params_.meas_k_range;

    metrics_pub_->publish(m);
  }

  // ---------- publish per-drone mode + done flags ----------
  void publishPerDroneModeAndFlags(const std::vector<int>& primary,
                                  const std::vector<int>& cap_per_drone,
                                  bool have_targets)
  {
    (void)cap_per_drone;

    for (size_t j = 0; j < drones_.size(); ++j) {
      const bool latched = latched_surveillance_[j];

      bool tracking = false;
      if (have_targets && !latched) {
        tracking = (primary.size() > j && primary[j] >= 0);
      }

      if (pub_mode_[j]) {
        std_msgs::msg::String s;
        s.data = tracking ? "TRACKING" : "SURVEILLANCE";
        pub_mode_[j]->publish(s);
      }

      if (pub_done_[j]) {
        std_msgs::msg::Bool b;
        b.data = latched;
        pub_done_[j]->publish(b);
      }

      if (pub_done_target_[j]) {
        std_msgs::msg::Int32 d;
        d.data = latched ? last_done_target_id_[j] : -1;
        pub_done_target_[j]->publish(d);
      }
    }
  }

  // ---------- publish hold outputs when N==0 ----------
  void publishHoldNoTargets()
  {
    const std::string frame = (last_header_.frame_id.empty() ? "world" : last_header_.frame_id);

    for (size_t j=0; j<drones_.size(); ++j) {
      if (pub_mode_[j]) {
        std_msgs::msg::String s;
        s.data = "SURVEILLANCE";
        pub_mode_[j]->publish(s);
      }

      msgs::AssignTarget tgt;
      tgt.target_id = -1;
      pub_assign_[j]->publish(tgt);

      geometry_msgs::msg::PoseStamped ps;
      ps.header = last_header_;
      ps.header.stamp = now();
      if (ps.header.frame_id.empty()) ps.header.frame_id = frame;
      ps.pose.position.x = drones_[j].position.x();
      ps.pose.position.y = drones_[j].position.y();
      ps.pose.position.z = drones_[j].position.z();
      ps.pose.orientation.w = 1.0;
      pub_assigned_pose_[j]->publish(ps);

      if (publish_qstar_dxdy_ && pub_qstar_dxdy_[j]) {
        geometry_msgs::msg::Vector3Stamped v;
        v.header.stamp = now();
        const std::string ns_clean = drone_ns_[j].empty() ? "" : drone_ns_[j].substr(1);
        v.header.frame_id = ns_clean.empty() ? "base_footprint" : (ns_clean + "/base_footprint");
        v.vector.x = 0.0; v.vector.y = 0.0; v.vector.z = 0.0;
        pub_qstar_dxdy_[j]->publish(v);
      }

      last_qstar_valid_[j] = false;
      last_primary_track_id_[j] = -1;
    }
  }

  // ---------- publish assignments + marker visualization ----------
  void publishAssignmentsAndMarkers(const std::vector<int>& primary)
  {
    visualization_msgs::msg::MarkerArray marr;
    {
      visualization_msgs::msg::Marker del;
      del.action = visualization_msgs::msg::Marker::DELETEALL;
      marr.markers.push_back(del);
    }

    const std::string frame = (last_header_.frame_id.empty() ? "world" : last_header_.frame_id);

    for (size_t j=0; j<primary.size(); ++j) {

      if (latched_surveillance_[j]) {
        msgs::AssignTarget tgt;
        tgt.target_id = -1;
        pub_assign_[j]->publish(tgt);

        geometry_msgs::msg::PoseStamped ps;
        ps.header = last_header_;
        ps.header.stamp = now();
        if (ps.header.frame_id.empty()) ps.header.frame_id = frame;

        ps.pose.position.x = drones_[j].position.x();
        ps.pose.position.y = drones_[j].position.y();
        ps.pose.position.z = drones_[j].position.z();
        ps.pose.orientation.w = 1.0;
        pub_assigned_pose_[j]->publish(ps);

        if (publish_qstar_dxdy_ && pub_qstar_dxdy_[j]) {
          geometry_msgs::msg::Vector3Stamped v;
          v.header.stamp = now();
          const std::string ns_clean = drone_ns_[j].empty() ? "" : drone_ns_[j].substr(1);
          v.header.frame_id = ns_clean.empty() ? "base_footprint" : (ns_clean + "/base_footprint");
          v.vector.x = 0.0; v.vector.y = 0.0; v.vector.z = 0.0;
          pub_qstar_dxdy_[j]->publish(v);
        }

        last_qstar_valid_[j] = false;
        continue;
      }

      msgs::AssignTarget tgt;
      tgt.target_id = -1;

      const bool has = (primary[j] >= 0 && primary[j] < (int)tracks_.size());
      if (has) {
        const auto & trk = tracks_[(size_t)primary[j]];

        // Extra safety: if this class is done, publish -1 (should already be blocked by cost matrix)
        if (trk.class_id >= 0 && done_class_ids_.count(trk.class_id) > 0) {
          pub_assign_[j]->publish(tgt);
          last_qstar_valid_[j] = false;
          continue;
        }

        geometry_msgs::msg::Pose qstar =
          chooseBestHoverPose(j, drones_[j].position, trk);

        last_qstar_pose_[j]  = qstar;
        last_qstar_valid_[j] = true;

        tgt.target_id = trk.id;
        tgt.hover_pose_world = qstar;
        tgt.min_alt = min_alt_;
        tgt.max_alt = max_alt_;
        tgt.safety_radius = r_safe_;
        tgt.timeout_s = 10.0;
        tgt.policy = "info_ring_frozen";

        pub_assign_[j]->publish(tgt);

        msgs::TrackedObstacle out;
        out.header = last_header_;
        out.header.stamp = now();
        if (out.header.frame_id.empty()) out.header.frame_id = frame;

        out.id = trk.id;
        out.position.x = trk.mean.x();
        out.position.y = trk.mean.y();
        out.position.z = trk.mean.z();

        out.covariance[0]=trk.covariance(0,0); out.covariance[1]=trk.covariance(0,1); out.covariance[2]=trk.covariance(0,2);
        out.covariance[3]=trk.covariance(1,0); out.covariance[4]=trk.covariance(1,1); out.covariance[5]=trk.covariance(1,2);
        out.covariance[6]=trk.covariance(2,0); out.covariance[7]=trk.covariance(2,1); out.covariance[8]=trk.covariance(2,2);

        out.class_id = trk.class_id;
        out.drone_ns = drone_ns_[j];
        pub_assigned_obstacle_[j]->publish(out);

        geometry_msgs::msg::PoseStamped ps;
        ps.header = out.header;
        ps.pose = qstar;
        pub_assigned_pose_[j]->publish(ps);

        if (publish_qstar_dxdy_ && pub_qstar_dxdy_[j]) {
          Eigen::Vector3d q(qstar.position.x, qstar.position.y, qstar.position.z);
          Eigen::Vector3d d = q - drones_[j].position;

          const double cy = std::cos(-drones_[j].yaw);
          const double sy = std::sin(-drones_[j].yaw);
          const double bx = cy*d.x() - sy*d.y();
          const double by = sy*d.x() + cy*d.y();

          geometry_msgs::msg::Vector3Stamped v;
          v.header.stamp = now();
          const std::string ns_clean = drone_ns_[j].empty() ? "" : drone_ns_[j].substr(1);
          v.header.frame_id = ns_clean.empty() ? "base_footprint" : (ns_clean + "/base_footprint");
          v.vector.x = bx;
          v.vector.y = by;
          v.vector.z = 0.0;
          pub_qstar_dxdy_[j]->publish(v);
        }

        visualization_msgs::msg::Marker line;
        line.header = out.header;
        line.ns = "assign_line";
        line.id = static_cast<int>(j);
        line.type = visualization_msgs::msg::Marker::LINE_LIST;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.03;
        line.color.a = 1.0; line.color.r = 1.0; line.color.g = 1.0; line.color.b = 1.0;
        geometry_msgs::msg::Point p_dr, p_tr;
        p_dr.x = drones_[j].position.x(); p_dr.y = drones_[j].position.y(); p_dr.z = drones_[j].position.z();
        p_tr.x = trk.mean.x(); p_tr.y = trk.mean.y(); p_tr.z = trk.mean.z();
        line.points.push_back(p_dr); line.points.push_back(p_tr);
        marr.markers.push_back(line);

        const double ring_z = use_fixed_altitude_ ? fixed_altitude_m_ : (trk.mean.z() + hover_params_.h);
        const int base_id = 10000 + (int)j * 1000;
        if (viz_ring_) {
          marr.markers.push_back(
            makeRingMarkerWorld(base_id + 0, frame, trk.mean, ring_z, hover_params_.r_h, hover_params_.l)
          );
        }
        if (viz_qstar_) {
          marr.markers.push_back(
            makeQstarMarkerWorld(base_id + 1, frame, qstar)
          );
        }

      } else {
        last_qstar_valid_[j] = false;
        pub_assign_[j]->publish(tgt);
      }
    }

    markers_pub_->publish(marr);
  }

  // ---------- termination handling + SURVEILLANCE latch ----------
  void checkTerminationAndHandle(const std::vector<int>& primary)
  {
    for (size_t j = 0; j < primary.size(); ++j) {

      if (latched_surveillance_[j]) continue;

      const int ti = primary[j];
      if (ti < 0 || ti >= (int)tracks_.size()) continue;

      const int target_id   = tracks_[(size_t)ti].id;
      const int target_class = tracks_[(size_t)ti].class_id;

      // If this class is already DONE by someone else, do nothing here.
      // The cost matrix will stop assigning it on the next tick.
      if (target_class >= 0 && done_class_ids_.count(target_class) > 0) continue;

      const bool at_circle = reachedHoverCircle(j, (size_t)ti);
      const bool term      = terminationSatisfiedForDrone(j, (size_t)ti);
      const bool best      = (!require_best_estimator_) ? true : isBestEstimatorForTarget(j, (size_t)ti);

      bool declare_done = false;
      if (require_reach_and_term_) declare_done = at_circle && term && best;
      else                        declare_done = term && best;

      if (!declare_done) continue;

      // GLOBAL DONE lockout by class_id (container identity)
      if (target_class >= 0) {
        done_class_ids_.insert(target_class);
      }

      latched_surveillance_[j] = true;
      last_done_target_id_[j]  = target_id;

      if (verbose_) {
        RCLCPP_WARN(get_logger(),
          "[DONE->LOCK] drone=%s DONE target_id=%d class_id=%d. Class is now blocked for ALL drones until /reset_tracking.",
          drone_ns_[j].c_str(), target_id, target_class);
      }
    }
  }

  // ---------- tick ----------
  void tick()
  {
    const size_t M = drones_.size();
    if (M == 0) return;

    for (size_t j = 0; j < M; ++j) if (!have_pose_[j]) return;

    const size_t N = tracks_.size();

    if (N == 0) {
      publishHoldNoTargets();

      std::vector<int> dummy_primary(M, -1);
      std::vector<int> dummy_cap(M, 0);
      publishPerDroneModeAndFlags(dummy_primary, dummy_cap, /*have_targets=*/false);

      printImportantPerDrone(dummy_primary);
      logCsvPerDrone(dummy_primary, /*Cptr=*/nullptr);
      return;
    }

    Eigen::MatrixXd C = buildCostMatrix();

    const int s = 0;
    const int t = 1 + (int)M + (int)N;
    const int total_nodes = t + 1;
    MinCostFlow mcf(total_nodes);

    std::vector<int> cap_per_drone(M, drone_capacity_);
    for (size_t j = 0; j < M; ++j) {
      if (latched_surveillance_[j]) cap_per_drone[j] = 0;
    }

    int total_capacity = 0;
    for (int c : cap_per_drone) total_capacity += c;
    int F = std::min<int>(total_capacity, (int)N);

    for (int j=0; j<(int)M; ++j)
      mcf.addEdge(s, 1+j, cap_per_drone[(size_t)j], 0.0);

    for (int j=0; j<(int)M; ++j)
      for (int i=0; i<(int)N; ++i)
        mcf.addEdge(1+j, 1+(int)M+i, 1, C(j,i));

    for (int i=0; i<(int)N; ++i)
      mcf.addEdge(1+(int)M+i, t, 1, 0.0);

    auto [flow_sent, flow_cost] = mcf.solve(s, t, F);
    (void)flow_sent;
    (void)flow_cost;

    std::vector<std::vector<int>> drone_to_targets(M);
    const auto & G = mcf.graph();
    for (int j=0; j<(int)M; ++j) {
      int dj = 1 + j;
      for (const auto& e : G[dj]) {
        if (e.to >= 1+(int)M && e.to < 1+(int)M+(int)N) {
          int i = e.to - (1 + (int)M);
          if (e.flow > 0) drone_to_targets[(size_t)j].push_back(i);
        }
      }
    }

    assignments_per_drone_.assign(M, {});
    for (size_t j=0; j<M; ++j) {
      auto & vec = drone_to_targets[j];
      std::sort(vec.begin(), vec.end(),
        [&](int ia, int ib){
          return C((int)j, ia) < C((int)j, ib);
        });
      assignments_per_drone_[j] = vec;
    }

    std::vector<int> primary(M, -1);
    for (size_t j=0; j<M; ++j) {
      primary[j] = assignments_per_drone_[j].empty() ? -1 : assignments_per_drone_[j].front();
    }

    // update stickiness memory by track_id (after solving)
    for (size_t j = 0; j < M; ++j) {
      if (primary[j] >= 0 && primary[j] < (int)tracks_.size()) {
        last_primary_track_id_[j] = tracks_[(size_t)primary[j]].id;
      } else {
        last_primary_track_id_[j] = -1;
      }
    }

    publishAssignmentsAndMarkers(primary);
    publishMetrics(primary, C);
    checkTerminationAndHandle(primary);
    publishPerDroneModeAndFlags(primary, cap_per_drone, /*have_targets=*/true);

    printImportantPerDrone(primary);
    logCsvPerDrone(primary, &C);
  }
};

// main
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AssignmentNode>());
  rclcpp::shutdown();
  return 0;
}
