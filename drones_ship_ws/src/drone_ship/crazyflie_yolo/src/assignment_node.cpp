#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>

#include <Eigen/Dense>
#include <unordered_map>
#include <regex>
#include <cmath>
#include <sstream>
#include <fstream>
#include <iomanip>  // pretty-print matrix

#include "crazyflie_yolo/hungarian_assignment.hpp"

// msgs in your package
#include "crazyflie_yolo/msg/tracked_obstacle.hpp"
#include "crazyflie_yolo/msg/tracked_obstacle_array.hpp"
#include "crazyflie_yolo/msg/assign_target.hpp"

namespace msgs = crazyflie_yolo::msg;
using rclcpp::QoS;

// Hungarian API from your header
using mr_assign::Drone;
using mr_assign::Track;
using mr_assign::Params;
using mr_assign::assign_drones_to_tracks;

// ------------------ helpers ------------------
static std::string ns_from_topic(const std::string& topic) {
  // Extract leading namespace: "/drone1/..." -> "/drone1"
  std::smatch m;
  static const std::regex re("^(/[^/]+)");
  if (std::regex_search(topic, m, re)) return m[1].str();
  return ""; // no namespace
}

static void print_matrix_labeled(const Eigen::MatrixXd& C,
                                 rclcpp::Logger logger,
                                 const std::vector<int>& col_to_track_id = {},
                                 const char* title = "C")
{
  RCLCPP_DEBUG(logger, "%s (%ldx%ld)", title, C.rows(), C.cols());

  std::stringstream hdr;
  hdr << "           ";
  for (int i = 0; i < C.cols(); ++i) {
    if (!col_to_track_id.empty())
      hdr << "  col" << i << "(id=" << col_to_track_id[i] << ")";
    else
      hdr << "  col" << i;
  }
  RCLCPP_DEBUG(logger, "%s", hdr.str().c_str());

  for (int r = 0; r < C.rows(); ++r) {
    std::stringstream row;
    row << "dr[" << r << "] :";
    for (int c = 0; c < C.cols(); ++c) {
      row << ' ' << std::fixed << std::setprecision(3) << std::setw(8) << C(r,c);
    }
    RCLCPP_DEBUG(logger, "%s", row.str().c_str());
  }
}

// ================== node =====================
class AssignmentNode : public rclcpp::Node {
public:
  AssignmentNode() : rclcpp::Node("assignment_node") {
    // -------- params --------
    odom_topics_   = declare_parameter<std::vector<std::string>>("drone_odom_topics", {});
    alloc_rate_hz_ = declare_parameter<double>("alloc_rate_hz", 1.0);

    // cost / gating
    params_.alpha          = declare_parameter<double>("alpha", 0.1);
    params_.beta           = declare_parameter<double>("beta",  1.0);
    params_.gamma          = declare_parameter<double>("gamma", 0.2);
    params_.r_safe         = declare_parameter<double>("r_safe", 0.5);
    params_.d_max          = declare_parameter<double>("d_max", 25.0);
    params_.maha_gate_sq   = declare_parameter<double>("maha_gate_sq", 9.0);
    params_.kappa          = declare_parameter<double>("kappa", 1000.0);
    params_.unassigned_penalty = declare_parameter<double>("unassigned_penalty", 25.0);

    // hover policy
    hover_radius_  = declare_parameter<double>("hover_radius", 1.5);
    hover_height_  = declare_parameter<double>("hover_height", 2.0);
    min_alt_       = declare_parameter<double>("min_alt",  1.0);
    max_alt_       = declare_parameter<double>("max_alt", 50.0);
    safety_radius_ = declare_parameter<double>("safety_radius", 0.5);
    timeout_s_     = declare_parameter<double>("timeout_s", 10.0);
    policy_str_    = declare_parameter<std::string>("policy", "topdown");

    // logging/diagnostics
    verbose_        = declare_parameter<bool>("verbose", false);
    log_costs_      = declare_parameter<bool>("log_costs", false);
    cost_csv_path_  = declare_parameter<std::string>("cost_csv_path", std::string(""));

    // assignment policy
    assignment_policy_   = declare_parameter<std::string>("assignment_policy", "many_to_one"); // "many_to_one"
    per_target_capacity_ = declare_parameter<int>("per_target_capacity", 1); // K for many_to_one

    // -------- subs/pubs --------
    drones_.resize(odom_topics_.size());
    have_pose_.assign(odom_topics_.size(), false);
    publishers_.resize(odom_topics_.size());

    for (size_t i = 0; i < odom_topics_.size(); ++i) {
      const auto& odom_topic = odom_topics_[i];

      // odometry subscriber
      auto sub = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, QoS(20),
        [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
          drones_[i].position = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
          );
          have_pose_[i] = true;

          RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "odom[%zu]: (%.2f, %.2f, %.2f)",
            i, drones_[i].position.x(), drones_[i].position.y(), drones_[i].position.z());
        });
      odom_subs_.push_back(sub);

      // per-drone assignment publisher on "<ns>/assign_target"
      auto ns = ns_from_topic(odom_topic);
      const std::string pub_topic = ns.empty() ? "assign_target" : (ns + "/assign_target");
      publishers_[i] = create_publisher<msgs::AssignTarget>(pub_topic, 10);

      RCLCPP_INFO(get_logger(), "Subscribed odom[%zu] '%s' -> publish '%s'",
                  i, odom_topic.c_str(), pub_topic.c_str());
    }

    // fused tracks subscriber
    tracks_sub_ = create_subscription<msgs::TrackedObstacleArray>(
      "fused_tracks", QoS(10),
      std::bind(&AssignmentNode::tracksCallback, this, std::placeholders::_1));

    // visualization (optional)
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("assignment_markers", 10);

    // timer loop
    const auto period = std::chrono::duration<double>(1.0 / std::max(alloc_rate_hz_, 1e-3));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AssignmentNode::tick, this));

    // startup banner
    RCLCPP_INFO(get_logger(),
      "assignment_node up. drones=%zu alloc=%.2f Hz | alpha=%.3f beta=%.3f gamma=%.3f "
      "r_safe=%.2f d_max=%.2f gate=%.2f kappa=%.1f unassigned=%.1f | policy=%s K=%d",
      odom_topics_.size(), alloc_rate_hz_,
      params_.alpha, params_.beta, params_.gamma, params_.r_safe, params_.d_max,
      params_.maha_gate_sq, params_.kappa, params_.unassigned_penalty,
      assignment_policy_.c_str(), per_target_capacity_);
  }

private:
  // -------- callbacks --------
  void tracksCallback(const msgs::TrackedObstacleArray::SharedPtr msg) {
    last_header_ = msg->header;
    tracks_.clear();
    ids_.clear();
    tracks_.reserve(msg->obstacles.size());
    ids_.reserve(msg->obstacles.size());

    RCLCPP_DEBUG(get_logger(), "fused_tracks: t=%u.%u count=%zu",
                 last_header_.stamp.sec, last_header_.stamp.nanosec, msg->obstacles.size());

    for (const auto& o : msg->obstacles) {
      Track t;
      t.mean = Eigen::Vector3d(o.position.x, o.position.y, o.position.z);

      Eigen::Matrix3d P;
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          P(r,c) = o.covariance[r*3 + c];

      t.covariance = P;
      t.measR = std::nullopt; // set if you have per-target measurement noise

      tracks_.push_back(t);
      ids_.push_back(o.id);
    }

    if (verbose_) {
      for (size_t i = 0; i < msg->obstacles.size(); ++i) {
        const auto& o = msg->obstacles[i];
        RCLCPP_DEBUG(get_logger(),
          "  obs[%zu] id=%d pos=(%.2f,%.2f,%.2f) cov=[%.3g .. %.3g .. %.3g]",
          i, o.id, o.position.x, o.position.y, o.position.z,
          o.covariance[0], o.covariance[4], o.covariance[8]);
      }
    }
  }

  void tick() {
    if (drones_.empty() || tracks_.empty()) {
      RCLCPP_DEBUG(get_logger(), "tick: waiting (drones=%zu, tracks=%zu)", drones_.size(), tracks_.size());
      return;
    }
    for (size_t i = 0; i < have_pose_.size(); ++i) {
      if (!have_pose_[i]) {
        RCLCPP_DEBUG(get_logger(), "tick: missing pose for drone[%zu]", i);
        return;
      }
    }

    RCLCPP_DEBUG(get_logger(), "tick: assigning (M=%zu, N=%zu)", drones_.size(), tracks_.size());

    // --- POLICY: many_to_one (duplicate columns so each real target can accept K drones) ---
    // This guarantees "one target per drone" when tracks_.size() * K >= drones_.size().
    std::vector<int> assignment;     // final per-drone (mapped to original column)
    Eigen::MatrixXd C_used;          // matrix we solved
    std::vector<int> backref;        // expanded column -> original track index
    std::vector<int> col_to_track_id;// for labeled printing

    if (assignment_policy_ == "many_to_one") {
      const int M = static_cast<int>(drones_.size());
      const int N = static_cast<int>(tracks_.size());
      const int K = std::max(1, per_target_capacity_);

      // Build expanded tracks
      std::vector<Track> tracks_expanded;
      tracks_expanded.reserve(N * K);
      backref.reserve(N * K);
      col_to_track_id.reserve(N * K);

      for (int i = 0; i < N; ++i) {
        for (int k = 0; k < K; ++k) {
          tracks_expanded.push_back(tracks_[i]);
          backref.push_back(i);
          col_to_track_id.push_back(ids_[i]); // real fused obstacle id
        }
      }

      // Compute cost and solve Hungarian
      Eigen::MatrixXd Cexp;
      assignment = assign_drones_to_tracks(drones_, tracks_expanded, params_, &Cexp);
      C_used = Cexp;

      if (log_costs_) {
        print_matrix_labeled(C_used, get_logger(), col_to_track_id, "C(expanded)");
        // optional CSV append
        if (!cost_csv_path_.empty()) append_cost_csv(C_used);
      }

      // Map expanded column -> original track column
      for (auto& col : assignment) {
        if (col >= 0 && col < (int)backref.size()) {
          col = backref[col];
        }
      }

    } else {
      // Fallback: standard one-to-one (will leave some drones UNASSIGNED if M > N)
      Eigen::MatrixXd C;
      assignment = assign_drones_to_tracks(drones_, tracks_, params_, &C);
      C_used = C;

      if (log_costs_) {
        print_matrix_labeled(C_used, get_logger(), /*col_to_track_id=*/{}, "C(raw)");
        if (!cost_csv_path_.empty()) append_cost_csv(C_used);
      }
    }

    // log assignment changes (mapped to original columns)
    if (assignment != last_assignment_) {
      if (verbose_) {
        std::ostringstream map_oss;
        map_oss << "Assignment changed: ";
        for (size_t j = 0; j < assignment.size(); ++j) {
          int ti = assignment[j];
          if (ti >= 0 && ti < static_cast<int>(ids_.size()))
            map_oss << "[drone " << j << " -> track " << ti << " (id=" << ids_[ti] << ")] ";
          else
            map_oss << "[drone " << j << " -> none] ";
        }
        RCLCPP_INFO(get_logger(), "%s", map_oss.str().c_str());
      }
      last_assignment_ = assignment;
    }

    // publish per-drone AssignTarget + RViz markers (uses original indices)
    publishAssignmentsAndMarkers(assignment);
  }

  // -------- helpers --------
  void publishAssignmentsAndMarkers(const std::vector<int>& assignment) {
    static constexpr double TWO_PI = 6.283185307179586;

    visualization_msgs::msg::MarkerArray marr;
    // clear previous markers
    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    marr.markers.push_back(del);

    for (size_t j = 0; j < assignment.size(); ++j) {
      msgs::AssignTarget tgt;
      tgt.target_id = -1; // unassigned by default

      const int ti = assignment[j];
      if (ti >= 0 && ti < static_cast<int>(tracks_.size())) {
        const auto &p = tracks_[ti].mean;

        // spread drones around the target with a circular offset
        const double angle = (assignment.size() > 0) ? (TWO_PI * static_cast<double>(j) / static_cast<double>(assignment.size())) : 0.0;
        const Eigen::Vector3d offset(hover_radius_ * std::cos(angle),
                                     hover_radius_ * std::sin(angle),
                                     hover_height_);
        const Eigen::Vector3d hover = p + offset;

        // fill message
        tgt.target_id = ids_[ti];
        tgt.hover_pose_world.position.x = hover.x();
        tgt.hover_pose_world.position.y = hover.y();
        tgt.hover_pose_world.position.z = hover.z();
        tgt.hover_pose_world.orientation.w = 1.0;  // simple top-down default
        tgt.min_alt = min_alt_;
        tgt.max_alt = max_alt_;
        tgt.safety_radius = safety_radius_;
        tgt.timeout_s = timeout_s_;
        tgt.policy = policy_str_;

        // RViz line: drone -> target
        visualization_msgs::msg::Marker line;
        line.header = last_header_;
        if (line.header.frame_id.empty()) line.header.frame_id = "world";
        line.ns = "assignments";
        line.id = static_cast<int>(j);
        line.type = visualization_msgs::msg::Marker::LINE_LIST;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.03;
        line.color.a = 1.0;
        line.color.r = 1.0;
        line.color.g = 1.0;
        line.color.b = 1.0;

        geometry_msgs::msg::Point p_dr, p_tr;
        p_dr.x = drones_[j].position.x();
        p_dr.y = drones_[j].position.y();
        p_dr.z = drones_[j].position.z();
        p_tr.x = p.x(); p_tr.y = p.y(); p_tr.z = p.z();

        line.points.push_back(p_dr);
        line.points.push_back(p_tr);
        marr.markers.push_back(line);

        if (verbose_) {
          RCLCPP_INFO(get_logger(),
            "assign drone[%zu] -> id=%d | hover=(%.2f, %.2f, %.2f) policy=%s",
            j, tgt.target_id,
            tgt.hover_pose_world.position.x,
            tgt.hover_pose_world.position.y,
            tgt.hover_pose_world.position.z,
            tgt.policy.c_str());
        }
      } else {
        if (verbose_) {
          RCLCPP_DEBUG(get_logger(), "assign drone[%zu] -> UNASSIGNED", j);
        }
      }

      publishers_[j]->publish(tgt);
    }

    markers_pub_->publish(marr);
  }

  void append_cost_csv(const Eigen::MatrixXd& C) {
    std::ofstream f(cost_csv_path_, std::ios::app); // append
    if (!f) {
      RCLCPP_WARN(get_logger(), "Failed to write cost CSV: %s", cost_csv_path_.c_str());
      return;
    }
    // timestamp, dims
    f << last_header_.stamp.sec << "." << last_header_.stamp.nanosec
      << "," << C.rows() << "x" << C.cols() << "\n";
    for (int r = 0; r < C.rows(); ++r) {
      for (int c = 0; c < C.cols(); ++c) {
        f << C(r,c) << (c + 1 < C.cols() ? "," : "\n");
      }
    }
  }

  // -------------- params --------------
  double alloc_rate_hz_{1.0};
  Params params_{};

  double hover_radius_{1.5}, hover_height_{2.0};
  double min_alt_{1.0}, max_alt_{50.0}, safety_radius_{0.5}, timeout_s_{10.0};
  std::string policy_str_{"topdown"};

  bool verbose_{false};
  bool log_costs_{false};
  std::string cost_csv_path_{};

  std::string assignment_policy_{"many_to_one"};
  int per_target_capacity_{1};

  // -------------- state ---------------
  std_msgs::msg::Header last_header_;
  std::vector<Drone> drones_;
  std::vector<bool> have_pose_;
  std::vector<Track> tracks_;
  std::vector<int> ids_;                 // track ids from fused stream
  std::vector<int> last_assignment_;     // for change-detection logging

  // -------------- ROS ------------------
  std::vector<std::string> odom_topics_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  rclcpp::Subscription<msgs::TrackedObstacleArray>::SharedPtr tracks_sub_;
  std::vector<rclcpp::Publisher<msgs::AssignTarget>::SharedPtr> publishers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AssignmentNode>());
  rclcpp::shutdown();
  return 0;
}
