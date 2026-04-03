#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "crazyflie_yolo/msg/tracked_obstacle.hpp"
#include "crazyflie_yolo/msg/tracked_obstacle_array.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>
#include <memory>
#include <chrono>
#include <unordered_map>

using vision_msgs::msg::Detection2DArray;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::CameraInfo;

using crazyflie_yolo::msg::TrackedObstacle;
using crazyflie_yolo::msg::TrackedObstacleArray;

struct Vec3 {
  double x{0.0}, y{0.0}, z{0.0};
};

static inline double dist3(const Vec3 &a, const Vec3 &b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

struct Track {
  int id{-1};
  int class_id{-1};
  std::string class_name;
  std::string drone_ns;

  // [px, py, pz, vx, vy, vz]
  Eigen::Matrix<double, 6, 1> x = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Identity();

  int missed{0};
  int age{0};
  int hits{0};
};

class SortTrackerNode : public rclcpp::Node {
public:
  SortTrackerNode() : Node("sort_tracker_node") {
    // -------------------------
    // Parameters
    // -------------------------
    yolo_topic_ = declare_parameter<std::string>("yolo_topic", "yolo_detections");
    disp_topic_ = declare_parameter<std::string>("disparity_topic", "stereo/disparity");
    cam_info_topic_ =
        declare_parameter<std::string>("camera_info_topic", "downward_left_camera/camera_info");
    output_topic_ =
        declare_parameter<std::string>("output_topic", "sort/tracked_obstacles_array");

    baseline_B_ = declare_parameter<double>("stereo_baseline_m", 0.10);
    min_disp_ = declare_parameter<double>("min_disparity", 0.5);
    max_depth_ = declare_parameter<double>("max_depth_m", 80.0);

    assoc_thresh_ = declare_parameter<double>("association_threshold", 12.0);
    max_missed_ = declare_parameter<int>("max_missed", 20);

    q_process_ = declare_parameter<double>("q_process", 1.5);
    r_meas_ = declare_parameter<double>("r_meas", 1.0);

    min_score_ = declare_parameter<double>("min_score", 0.70);

    // TF params
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    camera_frame_override_ = declare_parameter<std::string>("camera_frame", "");
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);

    // Sync / freshness params
    max_yolo_disp_dt_sec_ = declare_parameter<double>("max_yolo_disp_dt_sec", 0.20);
    max_data_stale_sec_ = declare_parameter<double>("max_data_stale_sec", 0.50);

    // Robust disparity sampling
    disparity_window_radius_ = declare_parameter<int>("disparity_window_radius", 3); // 7x7
    min_valid_disp_pixels_ = declare_parameter<int>("min_valid_disp_pixels", 3);

    // Optional fairness knob
    require_same_class_ = declare_parameter<bool>("require_same_class", false);

    // Publish filters (avoid noisy newborn tracks if desired)
    min_hits_to_publish_ = declare_parameter<int>("min_hits_to_publish", 1);

    // Debug params
    debug_ = declare_parameter<bool>("debug", true);
    debug_every_n_ticks_ = declare_parameter<int>("debug_every_n_ticks", 5);

    // -------------------------
    // TF
    // -------------------------
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // -------------------------
    // Subscribers / Publisher
    // -------------------------
    sub_yolo_ = create_subscription<Detection2DArray>(
        yolo_topic_, rclcpp::SensorDataQoS(),
        std::bind(&SortTrackerNode::yoloCb, this, std::placeholders::_1));

    sub_disp_ = create_subscription<Image>(
        disp_topic_, rclcpp::SensorDataQoS(),
        std::bind(&SortTrackerNode::dispCb, this, std::placeholders::_1));

    sub_cam_ = create_subscription<CameraInfo>(
        cam_info_topic_, rclcpp::SensorDataQoS(),
        std::bind(&SortTrackerNode::camCb, this, std::placeholders::_1));

    pub_ = create_publisher<TrackedObstacleArray>(output_topic_, rclcpp::QoS(10));

    timer_ = create_wall_timer(std::chrono::milliseconds(200),
                               std::bind(&SortTrackerNode::tick, this));

    RCLCPP_INFO(
        get_logger(),
        "SORT final | yolo=%s disp=%s cam=%s pub=%s | world=%s baseline=%.3f min_disp=%.3f "
        "max_depth=%.1f assoc=%.2f max_missed=%d min_score=%.2f win_r=%d sync=%.3fs stale=%.3fs",
        yolo_topic_.c_str(), disp_topic_.c_str(), cam_info_topic_.c_str(), output_topic_.c_str(),
        world_frame_.c_str(), baseline_B_, min_disp_, max_depth_, assoc_thresh_, max_missed_,
        min_score_, disparity_window_radius_, max_yolo_disp_dt_sec_, max_data_stale_sec_);
  }

private:
  // ============================================================
  // Callbacks
  // ============================================================
  void camCb(const CameraInfo::ConstSharedPtr msg) {
    if (msg->k[0] > 1e-6 && msg->k[4] > 1e-6) {
      fx_ = msg->k[0];
      fy_ = msg->k[4];
      cx_ = msg->k[2];
      cy_ = msg->k[5];
      have_cam_ = true;

      if (!msg->header.frame_id.empty()) {
        camera_info_frame_ = msg->header.frame_id;
      }

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                           "[SORT] CameraInfo | frame=%s fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
                           msg->header.frame_id.c_str(), fx_, fy_, cx_, cy_);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "[SORT] CameraInfo invalid K (fx/fy near zero).");
    }
  }

  void dispCb(const Image::ConstSharedPtr msg) {
    try {
      auto cvp = cv_bridge::toCvCopy(msg);
      last_disp_ = cvp->image;
      disp_stamp_ = msg->header.stamp;
      disp_frame_id_ = msg->header.frame_id;
      disp_encoding_ = msg->encoding;
      have_disp_ = !last_disp_.empty();

      // Cache one sample for debugging
      if (have_disp_ && last_disp_.type() == CV_32FC1 &&
          last_disp_.rows > 0 && last_disp_.cols > 0) {
        const int sx = std::clamp(last_disp_.cols / 2, 0, last_disp_.cols - 1);
        const int sy = std::clamp(last_disp_.rows / 2, 0, last_disp_.rows - 1);
        last_disp_center_sample_ = static_cast<double>(last_disp_.at<float>(sy, sx));
      }

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                           "[SORT] Disparity | frame=%s encoding=%s size=%dx%d type=%d",
                           msg->header.frame_id.c_str(), msg->encoding.c_str(), last_disp_.cols,
                           last_disp_.rows, last_disp_.type());
    } catch (const std::exception &e) {
      have_disp_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "[SORT] Failed to convert disparity image: %s", e.what());
    }
  }

  void yoloCb(const Detection2DArray::ConstSharedPtr msg) {
    last_yolo_ = msg;
    yolo_stamp_ = msg->header.stamp;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                         "[SORT] YOLO | n=%zu stamp=%.3f frame=%s",
                         msg->detections.size(),
                         rclcpp::Time(msg->header.stamp).seconds(),
                         msg->header.frame_id.c_str());
  }

  // ============================================================
  // Main processing loop
  // ============================================================
  void tick() {
    tick_count_++;

    if (!last_yolo_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "[SORT] Waiting for YOLO on: %s", yolo_topic_.c_str());
      return;
    }
    if (!have_cam_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "[SORT] Waiting for CameraInfo on: %s", cam_info_topic_.c_str());
      return;
    }
    if (!have_disp_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "[SORT] Waiting for disparity on: %s", disp_topic_.c_str());
      return;
    }

    const rclcpp::Time now = this->now();
    const rclcpp::Time t_yolo(last_yolo_->header.stamp);
    const rclcpp::Time t_disp(disp_stamp_);

    // Freshness guard
    const double yolo_stale = std::abs((now - t_yolo).seconds());
    const double disp_stale = std::abs((now - t_disp).seconds());
    if (yolo_stale > max_data_stale_sec_ || disp_stale > max_data_stale_sec_) {
      dbg_stale_skips_++;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[SORT] Skipping tick due to stale data | yolo_stale=%.3fs disp_stale=%.3fs (limit=%.3fs)",
          yolo_stale, disp_stale, max_data_stale_sec_);
      return;
    }

    // YOLO/disparity timestamp closeness guard
    const double yolo_disp_dt = std::abs((t_yolo - t_disp).seconds());
    if (yolo_disp_dt > max_yolo_disp_dt_sec_) {
      dbg_time_mismatch_skips_++;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1500,
          "[SORT] Skipping tick due to YOLO/disparity mismatch | |dt|=%.3fs > %.3fs",
          yolo_disp_dt, max_yolo_disp_dt_sec_);
      return;
    }

    const rclcpp::Time t = t_yolo;
    const double dt = (last_stamp_.nanoseconds() > 0)
                          ? std::max(1e-3, (t - last_stamp_).seconds())
                          : 0.2;
    last_stamp_ = t;

    resetTickDebug();

    // 1) Extract 3D world-frame measurements from 2D detections
    std::vector<Vec3> meas;
    std::vector<int> cls;
    std::vector<std::string> cls_name;
    std::vector<std::string> drone_ns;

    dbg_tick_total_yolo_ = last_yolo_->detections.size();

    for (const auto &det : last_yolo_->detections) {
      if (det.results.empty()) {
        dbg_tick_empty_results_++;
        continue;
      }

      const auto &res = det.results[0];
      const double score = res.hypothesis.score;
      if (score < min_score_) {
        dbg_tick_score_reject_++;
        continue;
      }
      dbg_tick_valid_score_++;

      // vision_msgs usually class_id is string
      const std::string class_id_str = res.hypothesis.class_id;
      int class_id = -1;
      try {
        class_id = std::stoi(class_id_str);
      } catch (...) {
        class_id = -1;
      }

      const std::string class_name = class_idStrToName(class_id_str);
      const std::string ns = normalizeNamespace(this->get_namespace());

      // bbox center (Detection2D center field variant support)
      double u = 0.0, v = 0.0;
      if (!extractBBoxCenter(det, u, v)) {
        dbg_tick_bbox_extract_fail_++;
        continue;
      }

      Vec3 p_world;
      if (!liftTo3DWorld(u, v, t, p_world)) {
        continue;
      }

      dbg_tick_valid_3d_++;
      meas.push_back(p_world);
      cls.push_back(class_id);
      cls_name.push_back(class_name);
      drone_ns.push_back(ns);
    }

    if (debug_ && (tick_count_ % std::max(1, debug_every_n_ticks_) == 0)) {
      printTickDebugHeader(dt, yolo_disp_dt);
    }

    // 2) Predict
    predictAll(dt);

    // 3) Associate + update
    greedyAssociateAndUpdate(meas, cls, cls_name, drone_ns);

    // 4) Prune
    const auto before_prune = tracks_.size();
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [this](const Track &tr) { return tr.missed > max_missed_; }),
                  tracks_.end());
    const auto after_prune = tracks_.size();
    const auto pruned_now = (before_prune >= after_prune) ? (before_prune - after_prune) : 0;
    dbg_tick_pruned_ = pruned_now;

    // 5) Publish
    publish(t);

    if (debug_ && (tick_count_ % std::max(1, debug_every_n_ticks_) == 0)) {
      RCLCPP_INFO(get_logger(),
                  "[SORT][assoc] meas=%zu updates=%zu new=%zu pruned=%zu tracks_after=%zu",
                  meas.size(), dbg_tick_updates_, dbg_tick_new_tracks_, dbg_tick_pruned_,
                  tracks_.size());

      RCLCPP_INFO(
          get_logger(),
          "[SORT][lift cumulative] ok=%zu | bounds=%zu type=%zu disp_invalid=%zu depth=%zu "
          "no_frame=%zu tf=%zu stale_skip=%zu time_mismatch_skip=%zu | last_disp_center=%.4f",
          dbg_ok_lift_, dbg_fail_bounds_, dbg_fail_type_, dbg_fail_disp_invalid_,
          dbg_fail_depth_, dbg_fail_no_frame_, dbg_fail_tf_, dbg_stale_skips_,
          dbg_time_mismatch_skips_, last_disp_center_sample_);
    }
  }

  // ============================================================
  // Helpers
  // ============================================================
  static std::string normalizeNamespace(const std::string &ns) {
    if (ns.empty()) return "";
    if (ns.front() == '/') return ns.substr(1);
    return ns;
  }

  std::string class_idStrToName(const std::string &class_id_str) const {
    // If your YOLO publishes names elsewhere, keep string as-is.
    // Here we just return raw string.
    return class_id_str;
  }

  bool extractBBoxCenter(const vision_msgs::msg::Detection2D &det, double &u, double &v) {
    // ROS2 vision_msgs variants may expose center as:
    // det.bbox.center.position.x / y  OR det.bbox.center.x / y
    // This code assumes your current message variant supports .position.x/.position.y
    try {
      u = det.bbox.center.position.x;
      v = det.bbox.center.position.y;
      return std::isfinite(u) && std::isfinite(v);
    } catch (...) {
      return false;
    }
  }

  bool getRobustDisparityAt(double u, double v, float &d_out) {
    if (last_disp_.empty()) return false;
    if (last_disp_.type() != CV_32FC1) return false;

    const int cx = static_cast<int>(std::round(u));
    const int cy = static_cast<int>(std::round(v));
    const int r = std::max(0, disparity_window_radius_);

    std::vector<float> vals;
    vals.reserve((2 * r + 1) * (2 * r + 1));

    for (int yy = cy - r; yy <= cy + r; ++yy) {
      if (yy < 0 || yy >= last_disp_.rows) continue;
      for (int xx = cx - r; xx <= cx + r; ++xx) {
        if (xx < 0 || xx >= last_disp_.cols) continue;
        const float d = last_disp_.at<float>(yy, xx);
        if (std::isfinite(d) && d >= static_cast<float>(min_disp_)) {
          vals.push_back(d);
        }
      }
    }

    dbg_last_valid_disp_count_ = vals.size();

    if (static_cast<int>(vals.size()) < min_valid_disp_pixels_) {
      return false;
    }

    const auto mid_it = vals.begin() + vals.size() / 2;
    std::nth_element(vals.begin(), mid_it, vals.end());
    d_out = *mid_it;
    return std::isfinite(d_out);
  }

  // ============================================================
  // 3D lifting + TF
  // ============================================================
  bool liftTo3DWorld(double u, double v, const rclcpp::Time &stamp, Vec3 &out_world) {
    const int x = static_cast<int>(std::round(u));
    const int y = static_cast<int>(std::round(v));

    if (x < 0 || y < 0 || x >= last_disp_.cols || y >= last_disp_.rows) {
      dbg_fail_bounds_++;
      return false;
    }

    if (last_disp_.type() != CV_32FC1) {
      dbg_fail_type_++;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "[SORT] Disparity type=%d (expected CV_32FC1=5). "
                           "Use raw stereo/disparity, not disparity_viz.",
                           last_disp_.type());
      return false;
    }

    float d = 0.0f;
    if (!getRobustDisparityAt(u, v, d)) {
      dbg_fail_disp_invalid_++;
      return false;
    }
    last_used_disparity_ = static_cast<double>(d);

    const double Z = (fx_ * baseline_B_) / static_cast<double>(d);
    if (!std::isfinite(Z) || Z <= 0.0 || Z > max_depth_) {
      dbg_fail_depth_++;
      return false;
    }

    const double X = (u - cx_) * Z / fx_;
    const double Y = (v - cy_) * Z / fy_;

    // Determine source frame
    std::string src_frame = camera_frame_override_;
    if (src_frame.empty()) {
      if (!disp_frame_id_.empty()) {
        src_frame = disp_frame_id_;
      } else if (!camera_info_frame_.empty()) {
        src_frame = camera_info_frame_;
      }
    }

    if (src_frame.empty()) {
      dbg_fail_no_frame_++;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "[SORT] No source frame for TF (disparity/camera_info frame_id empty).");
      return false;
    }
    last_src_frame_ = src_frame;

    geometry_msgs::msg::PointStamped p_src;
    p_src.header.stamp = stamp;
    p_src.header.frame_id = src_frame;
    p_src.point.x = X;
    p_src.point.y = Y;
    p_src.point.z = Z;

    // If source frame already equals world frame, avoid TF call
    if (src_frame == world_frame_) {
      out_world = Vec3{X, Y, Z};
      dbg_ok_lift_++;
      return true;
    }

    geometry_msgs::msg::PointStamped p_world_msg;
    try {
      p_world_msg =
          tf_buffer_->transform(p_src, world_frame_, tf2::durationFromSec(tf_timeout_sec_));
    } catch (const tf2::TransformException &ex) {
      dbg_fail_tf_++;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "[SORT] TF failed %s -> %s: %s",
                           src_frame.c_str(), world_frame_.c_str(), ex.what());
      return false;
    }

    out_world = Vec3{p_world_msg.point.x, p_world_msg.point.y, p_world_msg.point.z};
    dbg_ok_lift_++;
    return true;
  }

  // ============================================================
  // KF predict/update
  // ============================================================
  void predictAll(double dt) {
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;

    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    const double q = q_process_;
    Q(0, 0) = q * dt * dt;
    Q(1, 1) = q * dt * dt;
    Q(2, 2) = q * dt * dt;
    Q(3, 3) = q * dt;
    Q(4, 4) = q * dt;
    Q(5, 5) = q * dt;

    for (auto &tr : tracks_) {
      tr.x = F * tr.x;
      tr.P = F * tr.P * F.transpose() + Q;
      tr.missed++;
      tr.age++;
    }
  }

  void updateTrack(Track &tr, const Vec3 &z) {
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H(2, 2) = 1.0;

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * (r_meas_ * r_meas_);

    Eigen::Vector3d zv(z.x, z.y, z.z);
    Eigen::Vector3d innov = zv - H * tr.x;
    Eigen::Matrix3d S = H * tr.P * H.transpose() + R;

    const double detS = S.determinant();
    if (!std::isfinite(detS) || std::abs(detS) < 1e-12) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "[SORT] Innovation covariance singular, skip update.");
      return;
    }

    Eigen::Matrix<double, 6, 3> K = tr.P * H.transpose() * S.inverse();
    tr.x = tr.x + K * innov;
    tr.P = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * tr.P;
    tr.missed = 0;
    tr.hits++;
  }

  void greedyAssociateAndUpdate(const std::vector<Vec3> &meas,
                                const std::vector<int> &cls,
                                const std::vector<std::string> &cls_name,
                                const std::vector<std::string> &drone_ns) {
    std::vector<bool> used(meas.size(), false);

    // Track-first greedy assignment
    for (auto &tr : tracks_) {
      double best = std::numeric_limits<double>::infinity();
      int best_j = -1;

      Vec3 tp{tr.x(0), tr.x(1), tr.x(2)};
      for (int j = 0; j < static_cast<int>(meas.size()); ++j) {
        if (used[j]) continue;
        if (require_same_class_ && tr.class_id >= 0 && cls[j] >= 0 && tr.class_id != cls[j]) {
          continue;
        }
        const double d = dist3(tp, meas[j]);
        if (d < best) {
          best = d;
          best_j = j;
        }
      }

      if (best_j >= 0 && best <= assoc_thresh_) {
        updateTrack(tr, meas[best_j]);
        tr.class_id = cls[best_j];
        tr.class_name = cls_name[best_j];
        tr.drone_ns = drone_ns[best_j];
        used[best_j] = true;
        dbg_tick_updates_++;
      }
    }

    // Spawn new tracks for unused measurements
    for (int j = 0; j < static_cast<int>(meas.size()); ++j) {
      if (used[j]) continue;

      Track tr;
      tr.id = next_id_++;
      tr.class_id = cls[j];
      tr.class_name = cls_name[j];
      tr.drone_ns = drone_ns[j];

      tr.x.setZero();
      tr.x(0) = meas[j].x;
      tr.x(1) = meas[j].y;
      tr.x(2) = meas[j].z;

      tr.P = Eigen::Matrix<double, 6, 6>::Identity();
      tr.P.topLeftCorner<3, 3>() *= 6.0;
      tr.P.bottomRightCorner<3, 3>() *= 12.0;
      tr.missed = 0;
      tr.age = 1;
      tr.hits = 1;

      tracks_.push_back(tr);
      dbg_tick_new_tracks_++;
    }
  }

  // ============================================================
  // Publish
  // ============================================================
  void publish(const rclcpp::Time &stamp) {
    TrackedObstacleArray out;
    out.header.stamp = stamp;
    out.header.frame_id = world_frame_;

    out.obstacles.reserve(tracks_.size());

    size_t published_count = 0;
    for (const auto &tr : tracks_) {
      if (tr.hits < min_hits_to_publish_) continue;

      TrackedObstacle o;
      o.id = tr.id;
      o.class_id = tr.class_id;
      o.class_name = tr.class_name;
      o.drone_ns = tr.drone_ns;

      o.position.x = tr.x(0);
      o.position.y = tr.x(1);
      o.position.z = tr.x(2);

      for (int i = 0; i < 9; ++i) o.covariance[i] = 0.0;
      o.covariance[0] = tr.P(0, 0);
      o.covariance[4] = tr.P(1, 1);
      o.covariance[8] = tr.P(2, 2);

      out.obstacles.push_back(o);
      published_count++;
    }

    dbg_tick_published_ = published_count;
    pub_->publish(out);

    if (debug_ && (tick_count_ % std::max(1, debug_every_n_ticks_) == 0)) {
      RCLCPP_INFO(get_logger(),
                  "[SORT][pub] topic=%s n_tracks=%zu frame=%s stamp=%.3f",
                  output_topic_.c_str(), out.obstacles.size(), out.header.frame_id.c_str(),
                  rclcpp::Time(out.header.stamp).seconds());
    }
  }

  // ============================================================
  // Debug helpers
  // ============================================================
  void resetTickDebug() {
    dbg_tick_total_yolo_ = 0;
    dbg_tick_empty_results_ = 0;
    dbg_tick_score_reject_ = 0;
    dbg_tick_valid_score_ = 0;
    dbg_tick_bbox_extract_fail_ = 0;
    dbg_tick_valid_3d_ = 0;
    dbg_tick_updates_ = 0;
    dbg_tick_new_tracks_ = 0;
    dbg_tick_pruned_ = 0;
    dbg_tick_published_ = 0;
    dbg_last_valid_disp_count_ = 0;
  }

  void printTickDebugHeader(double dt, double yolo_disp_dt) {
    RCLCPP_INFO(
        get_logger(),
        "[SORT][tick %zu] dt=%.3f | yolo_total=%zu empty=%zu score_rej=%zu score_ok=%zu "
        "bbox_fail=%zu lift_ok=%zu | upd=%zu new=%zu pruned=%zu pub=%zu",
        tick_count_, dt, dbg_tick_total_yolo_, dbg_tick_empty_results_, dbg_tick_score_reject_,
        dbg_tick_valid_score_, dbg_tick_bbox_extract_fail_, dbg_tick_valid_3d_, dbg_tick_updates_,
        dbg_tick_new_tracks_, dbg_tick_pruned_, dbg_tick_published_);

    RCLCPP_INFO(
        get_logger(),
        "[SORT][cfg/runtime] ns=%s yolo=%s disp=%s cam=%s out=%s | disp_encoding=%s type=%d | "
        "src_frame(last)=%s world=%s baseline=%.4f fx=%.3f fy=%.3f min_disp=%.3f "
        "last_used_d=%.4f valid_disp_px=%zu | |t_yolo-t_disp|=%.3f",
        this->get_namespace(), yolo_topic_.c_str(), disp_topic_.c_str(), cam_info_topic_.c_str(),
        output_topic_.c_str(), disp_encoding_.c_str(), last_disp_.empty() ? -1 : last_disp_.type(),
        last_src_frame_.c_str(), world_frame_.c_str(), baseline_B_, fx_, fy_, min_disp_,
        last_used_disparity_, dbg_last_valid_disp_count_, yolo_disp_dt);

    if (dbg_tick_valid_3d_ == 0 && dbg_tick_valid_score_ > 0) {
      RCLCPP_WARN(get_logger(), "[SORT][depth stats this tick] no valid 3D lifts this tick.");
    }
  }

  // ============================================================
  // Params / topics
  // ============================================================
  std::string yolo_topic_;
  std::string disp_topic_;
  std::string cam_info_topic_;
  std::string output_topic_;

  double baseline_B_{0.10};
  double min_disp_{0.5};
  double max_depth_{80.0};

  double assoc_thresh_{12.0};
  int max_missed_{20};

  double q_process_{1.5};
  double r_meas_{1.0};
  double min_score_{0.70};

  // TF / sync params
  std::string world_frame_{"world"};
  std::string camera_frame_override_;
  double tf_timeout_sec_{0.05};
  double max_yolo_disp_dt_sec_{0.20};
  double max_data_stale_sec_{0.50};

  // Disparity robust sampling
  int disparity_window_radius_{3};
  int min_valid_disp_pixels_{3};

  // Association / publish options
  bool require_same_class_{false};
  int min_hits_to_publish_{1};

  // Debug params
  bool debug_{true};
  int debug_every_n_ticks_{5};

  // ============================================================
  // Camera intrinsics / frames
  // ============================================================
  bool have_cam_{false};
  double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};
  std::string camera_info_frame_;
  std::string disp_frame_id_;
  std::string disp_encoding_;

  // ============================================================
  // Latest inputs
  // ============================================================
  Detection2DArray::ConstSharedPtr last_yolo_;
  cv::Mat last_disp_;
  bool have_disp_{false};

  rclcpp::Time yolo_stamp_;
  rclcpp::Time disp_stamp_;
  rclcpp::Time last_stamp_;

  // ============================================================
  // ROS handles
  // ============================================================
  rclcpp::Subscription<Detection2DArray>::SharedPtr sub_yolo_;
  rclcpp::Subscription<Image>::SharedPtr sub_disp_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_cam_;
  rclcpp::Publisher<TrackedObstacleArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ============================================================
  // Tracks
  // ============================================================
  std::vector<Track> tracks_;
  int next_id_{1};

  // ============================================================
  // Debug counters
  // ============================================================
  size_t tick_count_{0};

  // Per-tick summaries
  size_t dbg_tick_total_yolo_{0};
  size_t dbg_tick_empty_results_{0};
  size_t dbg_tick_score_reject_{0};
  size_t dbg_tick_valid_score_{0};
  size_t dbg_tick_bbox_extract_fail_{0};
  size_t dbg_tick_valid_3d_{0};
  size_t dbg_tick_updates_{0};
  size_t dbg_tick_new_tracks_{0};
  size_t dbg_tick_pruned_{0};
  size_t dbg_tick_published_{0};
  size_t dbg_last_valid_disp_count_{0};

  // Cumulative failures / stats
  size_t dbg_fail_bounds_{0};
  size_t dbg_fail_type_{0};
  size_t dbg_fail_disp_invalid_{0};
  size_t dbg_fail_depth_{0};
  size_t dbg_fail_no_frame_{0};
  size_t dbg_fail_tf_{0};
  size_t dbg_ok_lift_{0};
  size_t dbg_stale_skips_{0};
  size_t dbg_time_mismatch_skips_{0};

  // Last runtime values
  double last_disp_center_sample_{0.0};
  double last_used_disparity_{0.0};
  std::string last_src_frame_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SortTrackerNode>());
  rclcpp::shutdown();
  return 0;
}