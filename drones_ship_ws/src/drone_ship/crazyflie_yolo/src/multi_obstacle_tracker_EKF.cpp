#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Custom messages used in your current stack
#include "crazyflie_yolo/msg/tracked_obstacle.hpp"
#include "crazyflie_yolo/msg/tracked_obstacle_array.hpp"

#include <Eigen/Dense>
#include <map>
#include <vector>
#include <cmath>
#include <limits>

using crazyflie_yolo::msg::TrackedObstacle;
using crazyflie_yolo::msg::TrackedObstacleArray;

/**
 * Multi-Obstacle Tracker (EKF, Constant-Velocity model)
 * -----------------------------------------------------
 * This node is a drop-in alternative to your measurement-only tracker.
 * It implements a full EKF per obstacle track with a 6‑state CV model:
 *   x = [px, py, pz, vx, vy, vz]^T
 *
 * Subscribes:
 *   - <detection_topic> (geometry_msgs/PointStamped): 3D detections in the world frame
 * Publishes:
 *   - tracked_obstacles_array (crazyflie_yolo/TrackedObstacleArray)
 *   - tracked_obstacles (geometry_msgs/PoseArray)
 *   - tracked_obstacle_markers (visualization_msgs/MarkerArray)
 *
 * Key parameters (declare via ROS params):
 *   - detection_topic (string, default: "/stereo/obstacle_point")
 *   - association_threshold (double, default: 30.0)          // Mahalanobis gate
 *   - r_x, r_y, r_z (double, default: 0.5 each)              // measurement noise variances
 *   - q_process (double, default: 0.5)                       // white-accel spectral density
 *   - init_var_pos (double, default: 2.0)                    // initial P for position diag
 *   - init_var_vel (double, default: 1.0)                    // initial P for velocity diag
 *   - track_prune_time (double, default: 3.0)                // seconds without update
 */
class MultiObstacleTrackerEKF : public rclcpp::Node
{
public:
  MultiObstacleTrackerEKF()
  : Node("multi_obstacle_tracker_ekf")
  {
    // --- Parameters ---
    detection_topic_      = this->declare_parameter<std::string>("detection_topic", "/stereo/obstacle_point");
    association_threshold_ = this->declare_parameter<double>("association_threshold", 30.0);

    r_x_ = this->declare_parameter<double>("r_x", 0.5);
    r_y_ = this->declare_parameter<double>("r_y", 0.5);
    r_z_ = this->declare_parameter<double>("r_z", 0.5);

    q_process_ = this->declare_parameter<double>("q_process", 0.5);

    init_var_pos_ = this->declare_parameter<double>("init_var_pos", 2.0);
    init_var_vel_ = this->declare_parameter<double>("init_var_vel", 1.0);

    track_prune_time_ = this->declare_parameter<double>("track_prune_time", 3.0);

    // --- Publishers ---
    pose_pub_   = this->create_publisher<geometry_msgs::msg::PoseArray>("tracked_obstacles", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tracked_obstacle_markers", 10);
    tracked_pub_ = this->create_publisher<TrackedObstacleArray>("tracked_obstacles_array", 10);

    // --- Subscriber ---
    detection_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      detection_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MultiObstacleTrackerEKF::detectionCallback, this, std::placeholders::_1));

    // --- Timer for pruning and publishing ---
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&MultiObstacleTrackerEKF::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "Multi-Obstacle Tracker EKF Initialized. Subscribed to %s", detection_topic_.c_str());
  }

private:
  struct ObstacleTrack
  {
    int id;
    Eigen::Matrix<double,6,1> x;   // [px, py, pz, vx, vy, vz]^T
    Eigen::Matrix<double,6,6> P;   // covariance
    rclcpp::Time last_update;      // last update time (stamp of last associated measurement)
  };

  // --- ROS I/O ---
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr detection_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<TrackedObstacleArray>::SharedPtr tracked_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- Parameters/storage ---
  std::string detection_topic_;
  double association_threshold_;
  double r_x_, r_y_, r_z_;
  double q_process_;
  double init_var_pos_, init_var_vel_;
  double track_prune_time_;

  std::map<int, ObstacleTrack> tracks_;
  int next_track_id_ = 0;

  // ================= Helper builders =================
  static Eigen::Matrix<double,6,6> makeF(double dt)
  {
    Eigen::Matrix<double,6,6> F = Eigen::Matrix<double,6,6>::Identity();
    F(0,3) = dt; F(1,4) = dt; F(2,5) = dt; // position depends on velocity
    return F;
  }

  Eigen::Matrix<double,6,6> makeQ(double dt) const
  {
    // White acceleration model with spectral density q_process_
    double q = q_process_;
    double dt2 = dt*dt;
    double dt3 = dt2*dt;

    Eigen::Matrix<double,6,6> Q = Eigen::Matrix<double,6,6>::Zero();
    Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    Q.topLeftCorner<3,3>()      = (dt3/3.0) * q * I3;
    Q.topRightCorner<3,3>()     = (dt2/2.0) * q * I3;
    Q.bottomLeftCorner<3,3>()   = (dt2/2.0) * q * I3;
    Q.bottomRightCorner<3,3>()  = (dt)      * q * I3;
    return Q;
  }

  Eigen::Matrix3d makeR() const
  {
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R(0,0) = r_x_; R(1,1) = r_y_; R(2,2) = r_z_;
    return R;
  }

  // ================= Core callbacks =================
  void detectionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    // Measurement in world frame
    Eigen::Vector3d z(msg->point.x, msg->point.y, msg->point.z);
    const rclcpp::Time stamp = this->now();



    // 1) Predict all tracks up to this stamp (lazy per-track prediction)
    //    We'll cache predicted (x,P) for association at this time.
    struct Predicted { Eigen::Matrix<double,6,1> x; Eigen::Matrix<double,6,6> P; double d2; int id; };

    std::vector<Predicted> candidates;
    candidates.reserve(tracks_.size());

    Eigen::Matrix<double,3,6> H = Eigen::Matrix<double,3,6>::Zero();
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity(); // measure position only
    Eigen::Matrix3d Rm = makeR();

    for (auto &kv : tracks_) {
      int id = kv.first;
      auto &tr = kv.second;

      double dt = (stamp - tr.last_update).seconds();
      if (dt < 0.0) dt = 0.0;        // guard
      if (dt > 2.0) dt = 2.0;        // cap dt for numerical stability

      Eigen::Matrix<double,6,6> F = makeF(dt);
      Eigen::Matrix<double,6,6> Q = makeQ(dt);

      Eigen::Matrix<double,6,1> x_pred = F * tr.x;
      Eigen::Matrix<double,6,6> P_pred = F * tr.P * F.transpose() + Q;

      Eigen::Vector3d y = z - H * x_pred;                         // innovation
      Eigen::Matrix3d  S = H * P_pred * H.transpose() + Rm;       // innovation cov
      double d2 = (y.transpose() * S.inverse() * y)(0,0);         // Mahalanobis

      candidates.push_back({x_pred, P_pred, d2, id});
    }

    // 2) Nearest neighbor association with gating
    int best_id = -1;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (const auto &c : candidates) {
      if (c.d2 < association_threshold_ && c.d2 < best_d2) {
        best_d2 = c.d2;
        best_id = c.id;
      }
    }

    if (best_id >= 0) {
      // 3) EKF update for the associated track using its predicted state
      auto &tr = tracks_.at(best_id);

      double dt = (stamp - tr.last_update).seconds();
      if (dt < 0.0) dt = 0.0;
      if (dt > 2.0) dt = 2.0;

      Eigen::Matrix<double,6,6> F = makeF(dt);
      Eigen::Matrix<double,6,6> Q = makeQ(dt);
      Eigen::Matrix<double,6,1> x_pred = F * tr.x;
      Eigen::Matrix<double,6,6> P_pred = F * tr.P * F.transpose() + Q;

      Eigen::Matrix<double,3,6> H = Eigen::Matrix<double,3,6>::Zero();
      H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
      Eigen::Matrix3d Rm = makeR();

      Eigen::Vector3d y = z - H * x_pred;
      Eigen::Matrix3d S = H * P_pred * H.transpose() + Rm;
      Eigen::Matrix<double,6,3> K = P_pred * H.transpose() * S.inverse();

      tr.x = x_pred + K * y;
      tr.P = (Eigen::Matrix<double,6,6>::Identity() - K * H) * P_pred;
      tr.last_update = this->now();

      RCLCPP_INFO(this->get_logger(), "[EKF] Updated Track %d | pos=(%.2f, %.2f, %.2f) vel=(%.2f, %.2f, %.2f) d2=%.2f",
                  tr.id, tr.x(0), tr.x(1), tr.x(2), tr.x(3), tr.x(4), tr.x(5), best_d2);
    } else {
      // 4) Create a new track if no association
      createTrack(z, stamp);
    }
  }

  void createTrack(const Eigen::Vector3d &z_world, const rclcpp::Time &stamp)
  {
    ObstacleTrack tr;
    tr.id = next_track_id_++;
    tr.x.setZero();
    tr.x.head<3>() = z_world; // start at measured position, zero velocity

    tr.P.setZero();
    tr.P.topLeftCorner<3,3>()     = init_var_pos_ * Eigen::Matrix3d::Identity();
    tr.P.bottomRightCorner<3,3>() = init_var_vel_ * Eigen::Matrix3d::Identity();

    tr.last_update = this->now();
    tracks_[tr.id] = tr;
    RCLCPP_INFO(this->get_logger(), "[EKF] Created Track %d at (%.2f, %.2f, %.2f)", tr.id, z_world(0), z_world(1), z_world(2));
  }

  // Timer: prune & publish
  void onTimer()
  {
    pruneOldTracks();
    publishTrackedObstacles();
  }

  void pruneOldTracks()
  {
    const rclcpp::Time now = this->now();
    std::vector<int> to_remove;
    for (const auto &kv : tracks_) {
      double dt = (now - kv.second.last_update).seconds();
      if (dt > track_prune_time_) {
        to_remove.push_back(kv.first);
      }
    }
    for (int id : to_remove) {
      tracks_.erase(id);
      RCLCPP_INFO(this->get_logger(), "[EKF] Pruned Track %d", id);
    }
  }

  void publishTrackedObstacles()
  {
    // PoseArray
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = this->now();
    pose_array.header.frame_id = "world";

    // MarkerArray
    visualization_msgs::msg::MarkerArray markers;

    // TrackedObstacleArray
    TrackedObstacleArray arr;
    arr.header = pose_array.header;

    int marker_id = 0;
    for (const auto &kv : tracks_) {
      const auto &tr = kv.second;

      // Pose
      geometry_msgs::msg::Pose p;
      p.position.x = tr.x(0);
      p.position.y = tr.x(1);
      p.position.z = tr.x(2);
      p.orientation.w = 1.0; // no orientation
      pose_array.poses.push_back(p);

      // TrackedObstacle (export only 3x3 position covariance block)
      TrackedObstacle tobs;
      tobs.id = tr.id;
      tobs.position.x = tr.x(0);
      tobs.position.y = tr.x(1);
      tobs.position.z = tr.x(2);
      tobs.header = arr.header;
      
      Eigen::Matrix3d Ppos = tr.P.topLeftCorner<3,3>();
      for (int i=0; i<3; ++i)
        for (int j=0; j<3; ++j)
          tobs.covariance[i*3 + j] = Ppos(i,j);
      arr.obstacles.push_back(tobs);

      // Visualization markers
      visualization_msgs::msg::Marker sphere;
      sphere.header = arr.header;
      sphere.ns = "tracked_obstacles";
      sphere.id = marker_id++;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose = p;
      sphere.scale.x = 0.5; sphere.scale.y = 0.5; sphere.scale.z = 0.5;
      sphere.color.a = 0.9; sphere.color.r = 0.2; sphere.color.g = 0.8; sphere.color.b = 0.2;
      markers.markers.push_back(sphere);

      visualization_msgs::msg::Marker label;
      label.header = arr.header;
      label.ns = "tracked_obstacles_text";
      label.id = marker_id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose = p;
      label.pose.position.z += 0.6;
      label.scale.z = 0.35;
      label.color.a = 1.0; label.color.r = 1.0; label.color.g = 1.0; label.color.b = 1.0;
      char buff[128];
      std::snprintf(buff, sizeof(buff), "ID:%d vx:%.2f vy:%.2f vz:%.2f", tr.id, tr.x(3), tr.x(4), tr.x(5));
      label.text = buff;
      markers.markers.push_back(label);
    }

    pose_pub_->publish(pose_array);
    marker_pub_->publish(markers);
    tracked_pub_->publish(arr);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiObstacleTrackerEKF>());
  rclcpp::shutdown();
  return 0;
}
