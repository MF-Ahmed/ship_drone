#include <chrono>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "builtin_interfaces/msg/duration.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "std_msgs/msg/color_rgba.hpp"

#include "crazyflie_yolo/msg/tracked_obstacle.hpp"
#include "crazyflie_yolo/msg/tracked_obstacle_array.hpp"

using crazyflie_yolo::msg::TrackedObstacle;
using crazyflie_yolo::msg::TrackedObstacleArray;

using namespace std::chrono_literals;

class MultiDroneFusionNode : public rclcpp::Node
{
public:
  MultiDroneFusionNode()
  : Node("multi_drone_fusion_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    //
    // 1) Parameters
    //
    drone_names_ = this->declare_parameter<std::vector<std::string>>(
      "drone_names", {"drone1", "drone2", "drone3"});

    topic_format_ = this->declare_parameter<std::string>(
      "topic_format", "/%s/tracked_obstacles_array");

    association_threshold_ = this->declare_parameter<double>(
      "association_threshold", 20.0);

    // ==========================
    // PAPER-STYLE PRUNING PARAMS
    // ==========================
    // Paper: stale track = no recent measurements
    // -> use last_update (last CI update / associated measurement time)
    prune_time_s_ = this->declare_parameter<double>(
      "prune_time_s", 0.5);

    // Paper: prune if covariance grows beyond threshold (loss of observability)
    // We implement two equivalent options (either triggers):
    //  - logdet(P_pos) > prune_cov_logdet_max
    //  - max_std(P_pos) > prune_cov_max_std_m
    prune_cov_logdet_max_ = this->declare_parameter<double>(
      "prune_cov_logdet_max", 12.0); // ~ very large uncertainty (default)
    prune_cov_max_std_m_ = this->declare_parameter<double>(
      "prune_cov_max_std_m", 8.0);   // meters (default)

    // Paper "completed target": terminate when uncertainty small enough.
    // In fusion node we can optionally prune such "done" tracks.
    prune_done_enabled_ = this->declare_parameter<bool>(
      "prune_done_enabled", false);
    prune_done_logdet_ = this->declare_parameter<double>(
      "prune_done_logdet", 0.0); // smaller => tighter (enable via prune_done_enabled)

    // ----------------------------------------------------
    // Keep old mahal params (NOT used for pruning anymore)
    // ----------------------------------------------------
    mahal_prune_threshold_ = this->declare_parameter<double>(
      "mahal_prune_threshold", 2.0);
    mahal_max_misses_ = this->declare_parameter<int>(
      "mahal_max_misses", 5);

    aquabot_frame_ = this->declare_parameter<std::string>(
      "aquabot_frame", "aquabot/base_link");

    world_frame_ = this->declare_parameter<std::string>(
      "world_frame", "world");

    // Snapshot log (fused state)
    snapshot_log_path_ = this->declare_parameter<std::string>(
      "log_path",
      "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs_fusion/fused_obstacles.csv");

    snapshot_log_append_ = this->declare_parameter<bool>(
      "log_append", false);

    // Fusion event log (per inbox obs association)
    event_log_path_ = this->declare_parameter<std::string>(
      "event_log_path",
      "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs_fusion/fusion_events.csv");

    event_log_append_ = this->declare_parameter<bool>(
      "event_log_append", false);

    // Prune event log (one row per pruned track)
    prune_log_path_ = this->declare_parameter<std::string>(
      "prune_log_path",
      "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs_fusion/prune_events.csv");

    prune_log_append_ = this->declare_parameter<bool>(
      "prune_log_append", false);

    // Stats log (cumulative counts)
    stats_log_path_ = this->declare_parameter<std::string>(
      "stats_log_path",
      "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs_fusion/fusion_stats.csv");

    stats_log_append_ = this->declare_parameter<bool>(
      "stats_log_append", false);

    // How often to flush cumulative stats
    stats_period_s_ = this->declare_parameter<double>(
      "stats_period_s", 1.0);

    debug_ = this->declare_parameter<bool>("debug", false);
    debug_rate_ms_ = this->declare_parameter<int>("debug_rate_ms", 2000);

    //
    // 2) Publishers
    //
    fused_array_pub_ =
      this->create_publisher<TrackedObstacleArray>("fused_tracked_obstacles_array", 10);
    fused_posearray_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>("fused_tracked_obstacles", 10);
    fused_markers_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("fused_tracked_obstacle_markers", 10);

    //
    // 3) Subscribers
    //
    for (const auto & drone_name : drone_names_) {
      std::string topic = formatTopic(topic_format_, drone_name);

      auto sub = this->create_subscription<TrackedObstacleArray>(
        topic,
        rclcpp::SensorDataQoS(),
        [this, drone_name](TrackedObstacleArray::ConstSharedPtr msg) {
          this->droneCallback(msg, drone_name);
        });

      subs_.push_back(sub);

      RCLCPP_INFO(this->get_logger(),
                  "Subscribed to %s for drone '%s'",
                  topic.c_str(),
                  drone_name.c_str());
    }

    //
    // 4) Timers
    //
    fuse_timer_ = this->create_wall_timer(
      200ms,  // 5 Hz
      std::bind(&MultiDroneFusionNode::runFusion, this));

    publish_timer_ = this->create_wall_timer(
      200ms,  // 5 Hz
      std::bind(&MultiDroneFusionNode::publishFusedObstacles, this));

    if (stats_period_s_ > 0.0) {
      stats_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(stats_period_s_)),
        std::bind(&MultiDroneFusionNode::flushCumulativeStats, this));
    }

    //
    // 5) Logs
    //
    openSnapshotLog();
    openEventLog();
    openPruneLog();
    openStatsLog();

    next_fused_id_ = 0;

    // Parameter live updates
    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MultiDroneFusionNode::onParams, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "MultiDroneFusionNode initialized.");
  }

  ~MultiDroneFusionNode() override
  {
    if (snapshot_log_.is_open()) snapshot_log_.close();
    if (event_log_.is_open())    event_log_.close();
    if (prune_log_.is_open())    prune_log_.close();
    if (stats_log_.is_open())    stats_log_.close();
  }

private:
  //
  // Fused track
  //
  struct FusedTrack
  {
    int id = -1;
    int class_id = -1;
    std::string class_name;
    std::set<std::string> drones_seen;

    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Matrix3d P   = Eigen::Matrix3d::Identity();

    // Paper-style staleness uses "last_update" = last associated measurement / CI update time
    rclcpp::Time last_update;

    // Legacy fields kept for logging compatibility (not used for pruning decisions anymore)
    rclcpp::Time last_supported;
    int mahal_misses = 0;
    double last_best_d2 = std::numeric_limits<double>::infinity();
  };

  //
  // Inbox item
  //
  struct InboxItem
  {
    std::string src_drone_ns;
    int src_track_id = -1;
    int class_id = -1;
    std::string class_name;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
  };

  // ---------------------------
  // CSV helpers
  // ---------------------------
  static std::string escapeCsv(const std::string& s)
  {
    std::string out;
    out.reserve(s.size() + 4);
    for (char ch : s) {
      if (ch == '"') out += "\"\"";
      else out += ch;
    }
    return out;
  }

  static std::string joinSet(const std::set<std::string>& s, const char sep = ';')
  {
    std::ostringstream oss;
    bool first = true;
    for (const auto& x : s) {
      if (!first) oss << sep;
      first = false;
      oss << x;
    }
    return oss.str();
  }

  static void inc(std::map<int, uint64_t>& m, int key, uint64_t v = 1) { m[key] += v; }

  static std::string serializeClassCounts(
    const std::map<int, uint64_t>& counts,
    const std::map<int, std::string>& names)
  {
    std::ostringstream oss;
    bool first = true;
    for (const auto& kv : counts) {
      const int cid = kv.first;
      const uint64_t c = kv.second;
      auto itn = names.find(cid);
      const std::string cname = (itn != names.end()) ? itn->second : "";
      if (!first) oss << ";";
      first = false;
      oss << cid << ":" << cname << "=" << c;
    }
    return oss.str();
  }

  // ---------------------------
  // Logs openers
  // ---------------------------
  void openSnapshotLog()
  {
    std::ios_base::openmode mode = std::ios_base::out;
    if (snapshot_log_append_) mode |= std::ios_base::app;
    snapshot_log_.open(snapshot_log_path_, mode);

    if (!snapshot_log_.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Could not open snapshot log at %s", snapshot_log_path_.c_str());
      return;
    }

    if (!snapshot_log_append_) {
      snapshot_log_
        << "stamp_ns,node_ns,src_drone_ns,fused_id,class_id,class_name,frame,"
        << "x,y,z,"
        << "cov00,cov01,cov02,cov10,cov11,cov12,cov20,cov21,cov22"
        << std::endl;
    }
  }

  void openEventLog()
  {
    std::ios_base::openmode mode = std::ios_base::out;
    if (event_log_append_) mode |= std::ios_base::app;
    event_log_.open(event_log_path_, mode);

    if (!event_log_.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Could not open fusion event log at %s", event_log_path_.c_str());
      return;
    }

    if (!event_log_append_) {
      event_log_
        << "stamp_ns,node_ns,src_drone_ns,src_track_id,class_id,class_name,"
        << "associated_track_id,is_new_track,mahalanobis_d2,association_threshold,"
        << "pre_x,pre_y,pre_z,meas_x,meas_y,meas_z,post_x,post_y,post_z,"
        << "pre_cov_det,meas_cov_det,post_cov_det,omega_used"
        << std::endl;
    }
  }

  void openPruneLog()
  {
    std::ios_base::openmode mode = std::ios_base::out;
    if (prune_log_append_) mode |= std::ios_base::app;
    prune_log_.open(prune_log_path_, mode);

    if (!prune_log_.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Could not open prune log at %s", prune_log_path_.c_str());
      return;
    }

    if (!prune_log_append_) {
      // Keep existing columns, append paper-style reasons & cov stats at end
      prune_log_
        << "stamp_ns,node_ns,"
        << "track_id,class_id,class_name,"
        << "drones_seen,"
        << "age_supported_s,mahal_misses,best_d2,"
        << "reason_time,reason_mahal,"
        << "prune_time_s,mahal_prune_threshold,mahal_max_misses,"
        // appended fields (paper-style)
        << "age_update_s,reason_stale,reason_cov,reason_done,"
        << "logdet_pos,max_std_pos,"
        << "prune_cov_logdet_max,prune_cov_max_std_m,"
        << "prune_done_enabled,prune_done_logdet"
        << std::endl;
    }
  }

  void openStatsLog()
  {
    std::ios_base::openmode mode = std::ios_base::out;
    if (stats_log_append_) mode |= std::ios_base::app;
    stats_log_.open(stats_log_path_, mode);

    if (!stats_log_.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Could not open stats log at %s", stats_log_path_.c_str());
      return;
    }

    if (!stats_log_append_) {
      stats_log_
        << "stamp_ns,node_ns,"
        << "raw_total_cum,pruned_total_cum,active_tracks,"
        << "raw_classes_cum,pruned_classes_cum";

      for (const auto& dn : drone_names_) {
        stats_log_
          << ",raw_total_cum_" << dn
          << ",pruned_total_cum_" << dn
          << ",raw_classes_cum_" << dn
          << ",pruned_classes_cum_" << dn;
      }
      stats_log_ << std::endl;
    }
  }

  // ---------------------------
  // Live parameter updates
  // ---------------------------
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  rcl_interfaces::msg::SetParametersResult
  onParams(const std::vector<rclcpp::Parameter>& params)
  {
    for (const auto& p : params) {
      const auto& name = p.get_name();
      if (name == "association_threshold") association_threshold_ = p.as_double();

      // paper-style prune params
      else if (name == "prune_time_s") prune_time_s_ = p.as_double();
      else if (name == "prune_cov_logdet_max") prune_cov_logdet_max_ = p.as_double();
      else if (name == "prune_cov_max_std_m") prune_cov_max_std_m_ = p.as_double();
      else if (name == "prune_done_enabled") prune_done_enabled_ = p.as_bool();
      else if (name == "prune_done_logdet") prune_done_logdet_ = p.as_double();

      // legacy (kept)
      else if (name == "mahal_prune_threshold") mahal_prune_threshold_ = p.as_double();
      else if (name == "mahal_max_misses") mahal_max_misses_ = p.as_int();

      else if (name == "debug") debug_ = p.as_bool();
      else if (name == "debug_rate_ms") debug_rate_ms_ = p.as_int();
    }

    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    res.reason = "ok";
    return res;
  }

  // ---------------------------
  // Intake callback
  // ---------------------------
  void droneCallback(const TrackedObstacleArray::ConstSharedPtr msg,
                     const std::string & drone_ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (debug_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), debug_rate_ms_,
        "[INTAKE] %s: received %zu obstacles",
        drone_ns.c_str(), msg->obstacles.size());
    }

    for (const auto & obs : msg->obstacles) {
      InboxItem item;
      item.src_drone_ns = drone_ns;
      item.src_track_id = obs.id;
      item.class_id = obs.class_id;
      item.class_name = obs.class_name;
      item.position = Eigen::Vector3d(obs.position.x, obs.position.y, obs.position.z);

      Eigen::Matrix3d Rm = Eigen::Matrix3d::Identity();
      Rm <<
        obs.covariance[0], obs.covariance[1], obs.covariance[2],
        obs.covariance[3], obs.covariance[4], obs.covariance[5],
        obs.covariance[6], obs.covariance[7], obs.covariance[8];

      item.cov = Rm;

      inbox_.push_back(item);
    }
  }

  // ---------------------------
  // Mahalanobis d2 helper: use S = P + R
  // ---------------------------
  static bool mahalanobis_d2_PR(const Eigen::Vector3d& x,
                               const Eigen::Matrix3d& P,
                               const Eigen::Vector3d& z,
                               const Eigen::Matrix3d& R,
                               double& out_d2)
  {
    const Eigen::Vector3d dz = z - x;
    Eigen::Matrix3d S = P + R;

    Eigen::Matrix3d S_inv;
    bool invertible = false;
    double detS = 0.0;
    S.computeInverseAndDetWithCheck(S_inv, detS, invertible);
    if (!invertible) {
      out_d2 = std::numeric_limits<double>::infinity();
      return false;
    }

    out_d2 = dz.transpose() * S_inv * dz;
    return true;
  }

  // ---------------------------
  // Robust logdet + max std for a 3x3 covariance
  // ---------------------------
  static void covStats(const Eigen::Matrix3d& Pin,
                       double& out_logdet,
                       double& out_max_std)
  {
    // symmetrize for safety
    Eigen::Matrix3d P = 0.5 * (Pin + Pin.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(P);
    if (es.info() != Eigen::Success) {
      out_logdet = std::numeric_limits<double>::infinity();
      out_max_std = std::numeric_limits<double>::infinity();
      return;
    }

    Eigen::Vector3d evals = es.eigenvalues();
    // clamp eigenvalues to avoid log(<=0)
    const double eps = 1e-12;
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(evals[i]) || evals[i] < eps) evals[i] = eps;
    }

    out_logdet = std::log(evals[0]) + std::log(evals[1]) + std::log(evals[2]);
    const double max_ev = std::max(evals[0], std::max(evals[1], evals[2]));
    out_max_std = std::sqrt(max_ev);
  }

  // ---------------------------
  // PAPER PRUNE:
  //  - stale: (now - last_update) > prune_time_s
  //  - diverged: logdet(P) > prune_cov_logdet_max OR max_std(P) > prune_cov_max_std_m
  //  - done (optional): logdet(P) <= prune_done_logdet (if enabled)
  // ---------------------------
  std::vector<FusedTrack> pruneTracksPaper(const rclcpp::Time & now)
  {
    std::vector<int> to_delete;
    to_delete.reserve(fused_tracks_.size());
    if (debug_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[PRUNE-CHECK] tracks=%zu prune_time_s=%.2f cov_logdet_max=%.2f cov_max_std=%.2f done_en=%d done_logdet=%.2f now=%.3f",
        fused_tracks_.size(),
        prune_time_s_,
        prune_cov_logdet_max_,
        prune_cov_max_std_m_,
        prune_done_enabled_ ? 1 : 0,
        prune_done_logdet_,
        now.seconds());
    }


    for (auto & kv : fused_tracks_) {
      const auto & ft = kv.second;

      const double age_update = (now - ft.last_update).seconds();

      double logdetP = 0.0, maxStd = 0.0;
      covStats(ft.P, logdetP, maxStd);

      const bool reason_stale = (age_update > prune_time_s_);
      const bool reason_cov   = (std::isfinite(logdetP) && logdetP > prune_cov_logdet_max_) ||
                                (std::isfinite(maxStd)  && maxStd  > prune_cov_max_std_m_);
      const bool reason_done  = (prune_done_enabled_ && std::isfinite(logdetP) && logdetP <= prune_done_logdet_);

      // Keep legacy fields for log line shape
      const double age_supported_legacy = (now - ft.last_supported).seconds();

      if (debug_) {

        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "  [PRUNE-CHECK] id=%d age_update=%.3f logdet=%.3f max_std=%.3f stale=%d cov=%d done=%d class=%d drones=%zu",
          ft.id,
          age_update,
          logdetP,
          maxStd,
          reason_stale ? 1 : 0,
          reason_cov ? 1 : 0,
          reason_done ? 1 : 0,
          ft.class_id,
          ft.drones_seen.size());
      } 

      if (reason_stale || reason_cov || reason_done) {
        to_delete.push_back(ft.id);
      }
    }

    std::vector<FusedTrack> removed;
    removed.reserve(to_delete.size());

    for (int id : to_delete) {
      auto it = fused_tracks_.find(id);
      if (it == fused_tracks_.end()) continue;

      const FusedTrack ft = it->second;

      const double age_update = (now - ft.last_update).seconds();

      double logdetP = 0.0, maxStd = 0.0;
      covStats(ft.P, logdetP, maxStd);

      const bool reason_stale = (age_update > prune_time_s_);
      const bool reason_cov   = (std::isfinite(logdetP) && logdetP > prune_cov_logdet_max_) ||
                                (std::isfinite(maxStd)  && maxStd  > prune_cov_max_std_m_);
      const bool reason_done  = (prune_done_enabled_ && std::isfinite(logdetP) && logdetP <= prune_done_logdet_);

      // legacy placeholders
      const double age_supported = (now - ft.last_supported).seconds();
      const bool reason_time_legacy  = false;
      const bool reason_mahal_legacy = false;

      // LOG prune event
      if (prune_log_.is_open()) {
        prune_log_
          << now.nanoseconds() << ","
          << this->get_namespace() << ","
          << ft.id << ","
          << ft.class_id << ","
          << "\"" << escapeCsv(ft.class_name) << "\"" << ","
          << "\"" << escapeCsv(joinSet(ft.drones_seen)) << "\"" << ","
          << age_supported << ","
          << ft.mahal_misses << ","
          << (std::isfinite(ft.last_best_d2) ? ft.last_best_d2 : -1.0) << ","
          << (reason_time_legacy ? 1 : 0) << ","
          << (reason_mahal_legacy ? 1 : 0) << ","
          << prune_time_s_ << ","
          << mahal_prune_threshold_ << ","
          << mahal_max_misses_ << ","
          // appended fields
          << age_update << ","
          << (reason_stale ? 1 : 0) << ","
          << (reason_cov ? 1 : 0) << ","
          << (reason_done ? 1 : 0) << ","
          << logdetP << ","
          << maxStd << ","
          << prune_cov_logdet_max_ << ","
          << prune_cov_max_std_m_ << ","
          << (prune_done_enabled_ ? 1 : 0) << ","
          << prune_done_logdet_
          << std::endl;
      }
      if (debug_) {
        RCLCPP_WARN(this->get_logger(),
                    "[PRUNE] Removing track %d (stale=%d cov=%d done=%d; age_update=%.3f logdet=%.3f max_std=%.3f)",
                    ft.id,
                    reason_stale ? 1 : 0,
                    reason_cov ? 1 : 0,
                    reason_done ? 1 : 0,
                    age_update, logdetP, maxStd);
      }
      
      removed.push_back(ft);
      fused_tracks_.erase(it);
    }

    return removed;
  }

  // ---------------------------
  // Fusion loop
  // ---------------------------
  void runFusion()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const rclcpp::Time now = this->now();

    if (debug_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), debug_rate_ms_,
        "[FUSION] inbox=%zu fused_tracks=%zu",
        inbox_.size(), fused_tracks_.size());
    }

    // Keep class names warm
    for (const auto& obs : inbox_) {
      if (!obs.class_name.empty()) class_names_[obs.class_id] = obs.class_name;
    }

    // 1) RAW cumulative stats (unchanged)
    for (const auto& obs : inbox_) {
      raw_total_cum_++;
      raw_total_cum_per_drone_[obs.src_drone_ns]++;

      inc(raw_class_cum_, obs.class_id, 1);
      inc(raw_class_cum_per_drone_[obs.src_drone_ns], obs.class_id, 1);

      if (!obs.class_name.empty()) class_names_[obs.class_id] = obs.class_name;
    }

    // 2) Associate + CI fuse (unchanged logic)
    for (const auto& obs : inbox_) {
      int best_track_id = -1;
      double best_d2 = std::numeric_limits<double>::infinity();

      // find best existing track (same class) by d2 using S=P+R
      for (auto& kv : fused_tracks_) {
        auto& ft = kv.second;
        if (ft.class_id != obs.class_id) continue;

        double d2 = std::numeric_limits<double>::infinity();
        if (!mahalanobis_d2_PR(ft.pos, ft.P, obs.position, obs.cov, d2)) continue;

        if (d2 < best_d2) {
          best_d2 = d2;
          best_track_id = ft.id;
        }
      }

      const bool pass_gate = (best_track_id >= 0 && best_d2 <= association_threshold_);

      bool created_new = false;
      int associated_id_for_log = -1;

      Eigen::Vector3d pre_pos = Eigen::Vector3d::Zero();
      Eigen::Matrix3d pre_cov = Eigen::Matrix3d::Identity();
      double pre_cov_det = 0.0;

      Eigen::Vector3d meas_pos = obs.position;
      Eigen::Matrix3d meas_cov = obs.cov;
      double meas_cov_det = meas_cov.determinant();

      Eigen::Vector3d post_pos = obs.position;
      Eigen::Matrix3d post_cov = meas_cov;
      double post_cov_det = meas_cov_det;

      double omega_used = 0.0;

      if (pass_gate) {
        auto& ft = fused_tracks_.at(best_track_id);

        pre_pos = ft.pos;
        pre_cov = ft.P;
        pre_cov_det = pre_cov.determinant();

        if (ft.class_name.empty()) ft.class_name = obs.class_name;

        // CI omega based on determinant heuristic (your approach)
        double det1 = pre_cov.determinant();
        double det2 = meas_cov.determinant();
        double denom = det1 + det2 + 1e-12;
        double omega = (denom > 0.0) ? (det2 / denom) : 0.5;
        omega = std::clamp(omega, 0.0, 1.0);
        omega_used = omega;

        Eigen::Matrix3d P1_inv, P2_inv;
        double detP1, detP2;
        bool inv_ok1 = false, inv_ok2 = false;

        pre_cov.computeInverseAndDetWithCheck(P1_inv, detP1, inv_ok1);
        meas_cov.computeInverseAndDetWithCheck(P2_inv, detP2, inv_ok2);

        if (!inv_ok1 || !inv_ok2) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Singular covariance in CI update, skipping merge for track %d",
            best_track_id);
        } else {
          Eigen::Matrix3d info = omega * P2_inv + (1.0 - omega) * P1_inv;
          Eigen::Matrix3d Pfused = info.inverse();

          Eigen::Vector3d xfused =
            Pfused * (omega * P2_inv * meas_pos + (1.0 - omega) * P1_inv * pre_pos);

          ft.pos = xfused;
          ft.P = Pfused;

          // IMPORTANT for paper pruning: last_update is the last time it got a measurement / was updated
          ft.last_update = now;

          ft.drones_seen.insert(obs.src_drone_ns);
          if (ft.class_name.empty()) ft.class_name = obs.class_name;

          // keep legacy fields coherent (but not used for pruning)
          ft.last_supported = now;
          ft.mahal_misses = 0;
          ft.last_best_d2 = best_d2;

          post_pos = ft.pos;
          post_cov = ft.P;
          post_cov_det = post_cov.determinant();
          associated_id_for_log = ft.id;
        }
      } else {
        FusedTrack nt;
        nt.id = next_fused_id_++;
        nt.class_id = obs.class_id;
        nt.class_name = obs.class_name;
        nt.pos = obs.position;
        nt.P = obs.cov;

        // IMPORTANT: last_update initialized now (fresh measurement)
        nt.last_update = now;

        // legacy
        nt.last_supported = now;
        nt.mahal_misses = 0;
        nt.last_best_d2 = 0.0;

        nt.drones_seen.insert(obs.src_drone_ns);

        fused_tracks_[nt.id] = nt;

        post_pos = nt.pos;
        post_cov = nt.P;
        post_cov_det = nt.P.determinant();
        associated_id_for_log = nt.id;
        created_new = true;
      }

      // warm class name map
      if (!obs.class_name.empty()) class_names_[obs.class_id] = obs.class_name;

      // event log
      if (event_log_.is_open()) {
        event_log_
          << now.nanoseconds() << ","
          << this->get_namespace() << ","
          << obs.src_drone_ns << ","
          << obs.src_track_id << ","
          << obs.class_id << ","
          << "\"" << escapeCsv(obs.class_name) << "\"" << ","
          << associated_id_for_log << ","
          << (created_new ? 1 : 0) << ","
          << best_d2 << ","
          << association_threshold_ << ","
          << pre_pos.x() << "," << pre_pos.y() << "," << pre_pos.z() << ","
          << meas_pos.x() << "," << meas_pos.y() << "," << meas_pos.z() << ","
          << post_pos.x() << "," << post_pos.y() << "," << post_pos.z() << ","
          << pre_cov_det << ","
          << meas_cov_det << ","
          << post_cov_det << ","
          << omega_used
          << std::endl;
      }
    }

    // 3) PAPER-STYLE PRUNE AFTER attempting measurement updates
    std::vector<FusedTrack> pruned = pruneTracksPaper(now);

    // 4) PRUNED cumulative stats (unchanged bookkeeping)
    for (const auto& ft : pruned) {
      pruned_total_cum_++;
      inc(pruned_class_cum_, ft.class_id, 1);

      if (!ft.class_name.empty()) class_names_[ft.class_id] = ft.class_name;

      // attribute prune to every drone that contributed to that fused track
      for (const auto& dn : ft.drones_seen) {
        pruned_total_cum_per_drone_[dn]++;
        inc(pruned_class_cum_per_drone_[dn], ft.class_id, 1);
      }
    }

    // clear inbox after processing
    inbox_.clear();
  }

  // ---------------------------
  // Periodic cumulative stats flush
  // ---------------------------
  void flushCumulativeStats()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!stats_log_.is_open()) return;

    const rclcpp::Time now = this->now();

    const uint64_t active_tracks = static_cast<uint64_t>(fused_tracks_.size());

    const std::string raw_cls   = serializeClassCounts(raw_class_cum_, class_names_);
    const std::string pruned_cls= serializeClassCounts(pruned_class_cum_, class_names_);

    stats_log_
      << now.nanoseconds() << ","
      << this->get_namespace() << ","
      << raw_total_cum_ << ","
      << pruned_total_cum_ << ","
      << active_tracks << ","
      << "\"" << escapeCsv(raw_cls) << "\"" << ","
      << "\"" << escapeCsv(pruned_cls) << "\"";

    for (const auto& dn : drone_names_) {
      uint64_t raw_dn = 0, pruned_dn = 0;

      auto itr = raw_total_cum_per_drone_.find(dn);
      if (itr != raw_total_cum_per_drone_.end()) raw_dn = itr->second;

      auto itp = pruned_total_cum_per_drone_.find(dn);
      if (itp != pruned_total_cum_per_drone_.end()) pruned_dn = itp->second;

      std::string raw_cls_dn, pruned_cls_dn;

      auto itrc = raw_class_cum_per_drone_.find(dn);
      if (itrc != raw_class_cum_per_drone_.end()) {
        raw_cls_dn = serializeClassCounts(itrc->second, class_names_);
      }

      auto itpc = pruned_class_cum_per_drone_.find(dn);
      if (itpc != pruned_class_cum_per_drone_.end()) {
        pruned_cls_dn = serializeClassCounts(itpc->second, class_names_);
      }

      stats_log_
        << "," << raw_dn
        << "," << pruned_dn
        << ",\"" << escapeCsv(raw_cls_dn) << "\""
        << ",\"" << escapeCsv(pruned_cls_dn) << "\"";
    }

    stats_log_ << std::endl;
  }

  // ---------------------------
  // Publish fused obstacles (snapshot + markers)
  // ---------------------------
  void publishFusedObstacles()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const rclcpp::Time now = this->now();

    bool have_aquabot_world = false;
    Eigen::Vector3d aquabot_world = Eigen::Vector3d::Zero();

    try {
      auto tf_stamped = tf_buffer_.lookupTransform(
        world_frame_,   // target
        aquabot_frame_, // source
        tf2::TimePointZero);

      aquabot_world = Eigen::Vector3d(
        tf_stamped.transform.translation.x,
        tf_stamped.transform.translation.y,
        tf_stamped.transform.translation.z);

      have_aquabot_world = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "TF lookup failed (%s -> %s): %s",
        aquabot_frame_.c_str(), world_frame_.c_str(), ex.what());
    }

    TrackedObstacleArray fused_msg;
    fused_msg.header.stamp = now;
    fused_msg.header.frame_id = world_frame_;

    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header = fused_msg.header;

    visualization_msgs::msg::MarkerArray marker_array_msg;

    builtin_interfaces::msg::Duration lifetime_msg;
    lifetime_msg.sec = 0;
    lifetime_msg.nanosec = 500000000u;

    for (auto & kv : fused_tracks_) {
      auto & ft = kv.second;

      std::string rep_drone = ft.drones_seen.empty()
        ? std::string("n/a")
        : *(ft.drones_seen.begin());

      TrackedObstacle fused_ob;
      fused_ob.id = ft.id;
      fused_ob.class_id = ft.class_id;
      fused_ob.class_name = ft.class_name;
      fused_ob.drone_ns = rep_drone;

      fused_ob.position.x = ft.pos.x();
      fused_ob.position.y = ft.pos.y();
      fused_ob.position.z = ft.pos.z();

      fused_ob.covariance[0] = ft.P(0,0);
      fused_ob.covariance[1] = ft.P(0,1);
      fused_ob.covariance[2] = ft.P(0,2);
      fused_ob.covariance[3] = ft.P(1,0);
      fused_ob.covariance[4] = ft.P(1,1);
      fused_ob.covariance[5] = ft.P(1,2);
      fused_ob.covariance[6] = ft.P(2,0);
      fused_ob.covariance[7] = ft.P(2,1);
      fused_ob.covariance[8] = ft.P(2,2);

      fused_msg.obstacles.push_back(fused_ob);

      geometry_msgs::msg::Pose p;
      p.position.x = ft.pos.x();
      p.position.y = ft.pos.y();
      p.position.z = ft.pos.z();
      p.orientation.w = 1.0;
      pose_array_msg.poses.push_back(p);

      double range_val = -1.0;
      double range_std = -1.0;
      if (have_aquabot_world) {
        Eigen::Vector3d diff = ft.pos - aquabot_world;
        range_val = diff.norm();
        if (range_val > 1e-6) {
          Eigen::Vector3d u = diff.normalized();
          double var_range = u.transpose() * ft.P * u;
          if (var_range < 0.0) var_range = 0.0;
          range_std = std::sqrt(var_range);
        } else {
          range_std = 0.0;
        }
      }

      auto base_color  = colorForClass(ft.class_id);
      auto final_color = tintForDrone(base_color, rep_drone);

      visualization_msgs::msg::Marker m;
      m.header = fused_msg.header;
      m.ns = "fused_obstacles_sphere";
      m.id = ft.id * 2;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = p;
      m.scale.x = 0.6;
      m.scale.y = 0.6;
      m.scale.z = 0.6;
      m.color = final_color;
      m.lifetime = lifetime_msg;
      marker_array_msg.markers.push_back(m);

      visualization_msgs::msg::Marker t;
      t.header = fused_msg.header;
      t.ns = "fused_obstacles_text";
      t.id = ft.id * 2 + 1;
      t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      t.action = visualization_msgs::msg::Marker::ADD;
      t.pose = p;
      t.pose.position.z += 1.0;
      t.scale.z = 0.5;
      t.color.r = 1.0;
      t.color.g = 1.0;
      t.color.b = 1.0;
      t.color.a = 1.0;
      t.lifetime = lifetime_msg;

      std::ostringstream oss;
      oss << "ftrk:" << ft.id
          << " cls:" << ft.class_id
          << " (" << (ft.class_name.empty() ? "?" : ft.class_name) << ")"
          << " dr:"  << rep_drone;
      if (have_aquabot_world) {
        oss << "\nR:" << std::fixed << std::setprecision(1)
            << range_val << "m"
            << " ±" << std::setprecision(2)
            << range_std << "m";
      } else {
        oss << "\nR:n/a";
      }
      t.text = oss.str();
      marker_array_msg.markers.push_back(t);

      // Snapshot log row
      if (snapshot_log_.is_open()) {
        snapshot_log_
          << now.nanoseconds() << ","
          << this->get_namespace() << ","
          << rep_drone << ","
          << ft.id << ","
          << ft.class_id << ","
          << "\"" << escapeCsv(ft.class_name) << "\"" << ","
          << world_frame_ << ","
          << ft.pos.x() << ","
          << ft.pos.y() << ","
          << ft.pos.z() << ","
          << ft.P(0,0) << ","
          << ft.P(0,1) << ","
          << ft.P(0,2) << ","
          << ft.P(1,0) << ","
          << ft.P(1,1) << ","
          << ft.P(1,2) << ","
          << ft.P(2,0) << ","
          << ft.P(2,1) << ","
          << ft.P(2,2)
          << std::endl;
      }
    }

    fused_array_pub_->publish(fused_msg);
    fused_posearray_pub_->publish(pose_array_msg);
    fused_markers_pub_->publish(marker_array_msg);
  }

  // ---------------------------
  // Coloring helpers
  // ---------------------------
  std_msgs::msg::ColorRGBA colorForClass(int class_id)
  {
    std_msgs::msg::ColorRGBA c;
    c.a = 0.9f;

    switch (class_id % 5) {
      case 0: c.r = 1.0f; c.g = 0.2f; c.b = 0.2f; break;
      case 1: c.r = 0.2f; c.g = 1.0f; c.b = 0.2f; break;
      case 2: c.r = 0.2f; c.g = 0.2f; c.b = 1.0f; break;
      case 3: c.r = 1.0f; c.g = 0.5f; c.b = 0.0f; break;
      case 4:
      default: c.r = 0.6f; c.g = 0.2f; c.b = 0.8f; break;
    }
    return c;
  }

  std_msgs::msg::ColorRGBA tintForDrone(
    const std_msgs::msg::ColorRGBA & base,
    const std::string & drone_ns)
  {
    std_msgs::msg::ColorRGBA out = base;

    float factor = 1.0f;
    if (!drone_ns.empty()) {
      char c = drone_ns.back();
      if (c == '1') factor = 1.0f;
      else if (c == '2') factor = 1.3f;
      else if (c == '3') factor = 0.7f;
      else factor = 1.0f;
    }

    out.r = std::min(1.0f, base.r * factor);
    out.g = std::min(1.0f, base.g * factor);
    out.b = std::min(1.0f, base.b * factor);
    out.a = base.a;
    return out;
  }

  static std::string formatTopic(const std::string & fmt, const std::string & drone_name)
  {
    std::string out = fmt;
    size_t pos = out.find("%s");
    if (pos != std::string::npos) out.replace(pos, 2, drone_name);
    return out;
  }

  // ---------------------------
  // Params
  // ---------------------------
  std::vector<std::string> drone_names_;
  std::string topic_format_;

  double association_threshold_{20.0};

  // paper-style pruning params
  double prune_time_s_{0.5};
  double prune_cov_logdet_max_{12.0};
  double prune_cov_max_std_m_{8.0};
  bool   prune_done_enabled_{false};
  double prune_done_logdet_{0.0};

  // legacy params (kept)
  double mahal_prune_threshold_{2.0};
  int    mahal_max_misses_{5};

  std::string aquabot_frame_;
  std::string world_frame_;

  std::string snapshot_log_path_;
  bool snapshot_log_append_{false};

  std::string event_log_path_;
  bool event_log_append_{false};

  std::string prune_log_path_;
  bool prune_log_append_{false};

  std::string stats_log_path_;
  bool stats_log_append_{false};
  double stats_period_s_{1.0};

  bool debug_{false};
  int  debug_rate_ms_{2000};

  // ---------------------------
  // ROS pubs/subs/timers
  // ---------------------------
  rclcpp::Publisher<TrackedObstacleArray>::SharedPtr fused_array_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr fused_posearray_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fused_markers_pub_;
  std::vector<rclcpp::Subscription<TrackedObstacleArray>::SharedPtr> subs_;

  rclcpp::TimerBase::SharedPtr fuse_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ---------------------------
  // Fusion state
  // ---------------------------
  std::map<int, FusedTrack> fused_tracks_;
  int next_fused_id_{0};

  std::vector<InboxItem> inbox_;

  // ---------------------------
  // Logging streams
  // ---------------------------
  std::ofstream snapshot_log_;
  std::ofstream event_log_;
  std::ofstream prune_log_;
  std::ofstream stats_log_;

  // ---------------------------
  // Cumulative stats
  // ---------------------------
  uint64_t raw_total_cum_{0};
  uint64_t pruned_total_cum_{0};

  std::unordered_map<std::string, uint64_t> raw_total_cum_per_drone_;
  std::unordered_map<std::string, uint64_t> pruned_total_cum_per_drone_;

  std::map<int, uint64_t> raw_class_cum_;
  std::map<int, uint64_t> pruned_class_cum_;

  std::unordered_map<std::string, std::map<int, uint64_t>> raw_class_cum_per_drone_;
  std::unordered_map<std::string, std::map<int, uint64_t>> pruned_class_cum_per_drone_;

  // class id -> name
  std::map<int, std::string> class_names_;

  std::mutex mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiDroneFusionNode>());
  rclcpp::shutdown();
  return 0;
}
