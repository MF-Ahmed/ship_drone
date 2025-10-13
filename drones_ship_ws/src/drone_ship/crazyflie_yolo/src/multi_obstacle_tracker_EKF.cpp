#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "crazyflie_yolo/msg/tracked_obstacle.hpp"
#include "crazyflie_yolo/msg/tracked_obstacle_array.hpp"
#include "crazyflie_yolo/msg/detection3_d_stamped.hpp"

#include <Eigen/Dense>
#include <map>
#include <unordered_map>
#include <vector>
#include <deque>
#include <set>
#include <sstream>
#include <algorithm>
#include <limits>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cmath>

using crazyflie_yolo::msg::TrackedObstacle;
using crazyflie_yolo::msg::TrackedObstacleArray;
using crazyflie_yolo::msg::Detection3DStamped;

class MultiObstacleTrackerEKF : public rclcpp::Node
{
public:
  MultiObstacleTrackerEKF() : Node("multi_obstacle_tracker_ekf")
  {
    // ---------- Params ----------
    detection_topic_        = declare_parameter<std::string>("detection_topic", "/detections_3d");
    association_threshold_  = declare_parameter<double>("association_threshold", 12.0);
    require_same_class_     = declare_parameter<bool>("require_same_class_for_assoc", true);

    r_x_ = declare_parameter<double>("r_x", 0.5);
    r_y_ = declare_parameter<double>("r_y", 0.5);
    r_z_ = declare_parameter<double>("r_z", 0.5);

    q_process_    = declare_parameter<double>("q_process", 1.0);
    init_var_pos_ = declare_parameter<double>("init_var_pos", 2.0);
    init_var_vel_ = declare_parameter<double>("init_var_vel", 1.0);
    track_prune_time_ = declare_parameter<double>("track_prune_time", 3.0);

    // RViz / appearance
    sphere_scale_   = declare_parameter<double>("sphere_scale", 0.55);
    label_scale_    = declare_parameter<double>("label_scale", 0.38);
    marker_life_s_  = declare_parameter<double>("marker_lifetime", 0.6);
    draw_trail_     = declare_parameter<bool>("draw_trail", true);
    trail_len_      = declare_parameter<int>("trail_len", 40);

    // Covariance viz
    show_cov_axes_       = declare_parameter<bool>("show_cov_axes", true);
    sigma_axes_          = declare_parameter<double>("sigma_axes", 2.0);
    axes_width_          = declare_parameter<double>("axes_width", 0.05);
    show_cov_xy_ellipse_ = declare_parameter<bool>("show_cov_xy_ellipse", true);
    sigma_ellipse_       = declare_parameter<double>("sigma_ellipse", 2.0);
    ellipse_points_      = declare_parameter<int>("ellipse_points", 64);
    ellipse_width_       = declare_parameter<double>("ellipse_width", 0.03);

    // Z clamp (planar)
    clamp_z_  = declare_parameter<bool>("clamp_z", false);
    ground_z_ = declare_parameter<double>("ground_z", 0.0);

    // Class allow-list
    std::string allowed_classes_str = declare_parameter<std::string>("allowed_classes", "");
    {
      std::stringstream ss(allowed_classes_str);
      for (std::string tok; std::getline(ss, tok, ','); ) {
        tok.erase(std::remove_if(tok.begin(), tok.end(), ::isspace), tok.end());
        if (!tok.empty()) { try { allowed_classes_.insert(std::stoi(tok)); } catch (...) {} }
      }
      if (allowed_classes_.empty()) RCLCPP_INFO(get_logger(), "allowed_classes: <all>");
      else {
        std::string dbg; for (auto c : allowed_classes_) dbg += (dbg.empty()?"":",")+std::to_string(c);
        RCLCPP_INFO(get_logger(), "allowed_classes: {%s}", dbg.c_str());
      }
    }

    // ---------- GT subscriptions & logging ----------
    log_dir_ = declare_parameter<std::string>(
      "log_dir",
      "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs");

    // Aquabot GT
    gt_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
      "/aquabot/odometry", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m){ this->gtOdomCb(m, "aquabot"); }));

    // Container GTs
    int gt_count = declare_parameter<int>("gt_container_count", 5);
    for (int i=1;i<=gt_count;++i) {
      std::string name = "container" + std::to_string(i);
      gt_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        "/" + name + "/odometry", rclcpp::SensorDataQoS(),
        [this, name](nav_msgs::msg::Odometry::ConstSharedPtr m){ this->gtOdomCb(m, name); }));
    }

    openCsv();

    // ---------- Publishers / Subscriber ----------
    pose_pub_    = create_publisher<geometry_msgs::msg::PoseArray>("tracked_obstacles", rclcpp::QoS(10));
    marker_pub_  = create_publisher<visualization_msgs::msg::MarkerArray>("tracked_obstacle_markers", rclcpp::QoS(10));
    tracked_pub_ = create_publisher<TrackedObstacleArray>("tracked_obstacles_array", rclcpp::QoS(10));

    errors_pub_  = create_publisher<std_msgs::msg::Float32MultiArray>("/tracking_eval/errors", rclcpp::QoS(10));
    report_pub_  = create_publisher<std_msgs::msg::String>("/tracking_eval/report", rclcpp::QoS(10));

    detection_sub_ = create_subscription<Detection3DStamped>(
      detection_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MultiObstacleTrackerEKF::detectionCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&MultiObstacleTrackerEKF::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "EKF on %s | clamp_z=%s z=%.2f | cov_axes=%s(%.1fσ) ellipse=%s(%.1fσ) | logs=%s",
      detection_topic_.c_str(), clamp_z_?"true":"false", ground_z_,
      show_cov_axes_?"on":"off", sigma_axes_, show_cov_xy_ellipse_?"on":"off", sigma_ellipse_,
      log_path_.c_str());
  }

private:
  // ---------- Coloring ----------
  struct RGB { double r,g,b; };
  static constexpr std::array<RGB,5> kClassPalette = {{
    {1.00, 0.00, 0.00},  // 0 red
    {0.00, 0.80, 0.80},  // 1 cyan
    {0.80, 0.00, 0.80},  // 2 magenta
    {1.00, 1.00, 0.00},  // 3 yellow
    {0.00, 1.00, 0.00},  // 4 green
  }};
  static inline RGB colorForClass(int cls) {
    if (cls >= 0 && cls < (int)kClassPalette.size()) return kClassPalette[cls];
    return {0.6,0.6,0.6};
  }
  static inline int droneIndexFromNS(std::string ns) {
    if (!ns.empty() && ns.front()=='/') ns.erase(0,1);
    if (ns.rfind("drone2",0)==0) return 2;
    if (ns.rfind("drone3",0)==0) return 3;
    return 1;
  }
  static inline RGB tintByDrone(const RGB& base, const std::string& ns) {
    int idx = droneIndexFromNS(ns);
    if (idx==2) return { (base.r+1.0)*0.5, (base.g+1.0)*0.5, (base.b+1.0)*0.5 };
    if (idx==3) return { base.r*0.65, base.g*0.65, base.b*0.65 };
    return base;
  }
  static inline RGB colorByClassAndDrone(int cls, const std::string& ns) {
    return tintByDrone(colorForClass(cls), ns);
  }

  // ---------- Track ----------
  struct ObstacleTrack {
    int id;
    int class_id;
    std::string drone_ns;
    Eigen::Matrix<double,6,1> x; // [px,py,pz,vx,vy,vz]
    Eigen::Matrix<double,6,6> P;
    rclcpp::Time last_update;
    std::deque<Eigen::Vector3d> trail;
  };

  // ---------- ROS ----------
  rclcpp::Subscription<Detection3DStamped>::SharedPtr detection_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> gt_subs_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<TrackedObstacleArray>::SharedPtr tracked_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr errors_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            report_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---------- Params/state ----------
  std::string detection_topic_;
  double association_threshold_;
  bool   require_same_class_;

  double r_x_, r_y_, r_z_;
  double q_process_;
  double init_var_pos_, init_var_vel_;
  double track_prune_time_;

  // appearance
  double sphere_scale_, label_scale_, marker_life_s_;
  bool   draw_trail_;
  int    trail_len_;

  // covariance viz
  bool   show_cov_axes_;
  double sigma_axes_;
  double axes_width_;
  bool   show_cov_xy_ellipse_;
  double sigma_ellipse_;
  int    ellipse_points_;
  double ellipse_width_;

  // planar clamp
  bool   clamp_z_;
  double ground_z_;

  // GT + logging
  std::unordered_map<std::string, std::array<double,3>> gt_; // name -> (x,y,z)
  std::string log_dir_;
  std::string log_path_;
  std::ofstream csv_;

  std::set<int> allowed_classes_;
  std::map<int,ObstacleTrack> tracks_;
  int next_track_id_ = 0;

  // ---------- EKF helpers ----------
  static Eigen::Matrix<double,6,6> makeF(double dt) {
    Eigen::Matrix<double,6,6> F = Eigen::Matrix<double,6,6>::Identity();
    F(0,3)=dt; F(1,4)=dt; F(2,5)=dt;
    return F;
  }
  Eigen::Matrix<double,6,6> makeQ(double dt) const {
    double q = q_process_;
    double dt2 = dt*dt, dt3 = dt2*dt;
    Eigen::Matrix<double,6,6> Q = Eigen::Matrix<double,6,6>::Zero();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Q.topLeftCorner<3,3>()      = (dt3/3.0)*q*I;
    Q.topRightCorner<3,3>()     = (dt2/2.0)*q*I;
    Q.bottomLeftCorner<3,3>()   = (dt2/2.0)*q*I;
    Q.bottomRightCorner<3,3>()  = (dt)*q*I;
    return Q;
  }
  Eigen::Matrix3d makeR() const {
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R(0,0)=r_x_; R(1,1)=r_y_; R(2,2)=r_z_;
    return R;
  }

  // ---------- GT + CSV ----------
  void gtOdomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg, const std::string& name) {
    gt_[name] = { msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z };
  }

  void openCsv() {
    std::filesystem::create_directories(log_dir_);
    // file name
    std::time_t t = std::time(nullptr);
    std::tm tm{}; localtime_r(&t,&tm);
    std::ostringstream oss;
    oss << "tracking_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
    log_path_ = (std::filesystem::path(log_dir_) / oss.str()).string();

    csv_.open(log_path_, std::ios::out);
    if (!csv_) {
      RCLCPP_WARN(get_logger(), "Could not open CSV: %s", log_path_.c_str());
      return;
    }
    // header (ADDED Aquabot ranges and range error columns)
    csv_ << "stamp,track_id,class,ns,"
         << "est_x,est_y,est_z,"
         << "gt_name,gt_x,gt_y,gt_z,"
         << "err_x,err_y,err_z,err_norm,"
         << "aq_range_est,aq_range_gt,aq_range_err\n";
    csv_.flush();
  }

  // ---------- Core  ----------
  void detectionCallback(const Detection3DStamped::SharedPtr msg)
  {
    if (!allowed_classes_.empty() && allowed_classes_.count(msg->class_id)==0) return;

    rclcpp::Time stamp = msg->header.stamp;
    if (stamp.nanoseconds()==0) stamp = now();

    // measurement
    Eigen::Vector3d z(msg->position.x, msg->position.y, msg->position.z);
    if (clamp_z_) z(2) = ground_z_;

    // Predict each track to this stamp and compute Mahalanobis distance
    struct Pred { int id; Eigen::Matrix<double,6,1> x; Eigen::Matrix<double,6,6> P; double d2; };
    std::vector<Pred> cand; cand.reserve(tracks_.size());

    Eigen::Matrix<double,3,6> H = Eigen::Matrix<double,3,6>::Zero();
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Rm = makeR();

    for (auto &kv : tracks_) {
      auto &tr = kv.second;
      if (require_same_class_ && tr.class_id != msg->class_id) continue;

      double dt = (stamp - tr.last_update).seconds();
      dt = std::clamp(dt, 0.0, 2.0);

      Eigen::Matrix<double,6,6> F = makeF(dt);
      Eigen::Matrix<double,6,6> Q = makeQ(dt);
      Eigen::Matrix<double,6,1> x_pred = F*tr.x;
      Eigen::Matrix<double,6,6> P_pred = F*tr.P*F.transpose() + Q;

      Eigen::Vector3d y = z - H*x_pred;
      Eigen::Matrix3d  S = H*P_pred*H.transpose() + Rm;
      double d2 = (y.transpose() * S.inverse() * y)(0,0);

      cand.push_back({tr.id, x_pred, P_pred, d2});
    }

    // Nearest neighbor with gate
    int best_id = -1; double best_d2 = std::numeric_limits<double>::infinity();
    for (const auto &c : cand) if (c.d2 < association_threshold_ && c.d2 < best_d2) { best_d2 = c.d2; best_id = c.id; }

    if (best_id >= 0) {
      // Update associated track
      auto &tr = tracks_.at(best_id);

      double dt = (stamp - tr.last_update).seconds();
      dt = std::clamp(dt, 0.0, 2.0);

      Eigen::Matrix<double,6,6> F = makeF(dt);
      Eigen::Matrix<double,6,6> Q = makeQ(dt);
      Eigen::Matrix<double,6,1> x_pred = F*tr.x;
      Eigen::Matrix<double,6,6> P_pred = F*tr.P*F.transpose() + Q;

      Eigen::Matrix<double,3,6> H = Eigen::Matrix<double,3,6>::Zero();
      H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
      Eigen::Matrix3d Rm = makeR();

      Eigen::Vector3d y = z - H*x_pred;
      Eigen::Matrix3d S = H*P_pred*H.transpose() + Rm;
      Eigen::Matrix<double,6,3> K = P_pred * H.transpose() * S.inverse();

      tr.x = x_pred + K*y;
      tr.P = (Eigen::Matrix<double,6,6>::Identity() - K*H) * P_pred;
      tr.last_update = stamp;

      if (clamp_z_) { tr.x(2) = ground_z_; tr.P(2,2) = std::min(tr.P(2,2), init_var_pos_); }

      if (draw_trail_) {
        tr.trail.push_back(tr.x.head<3>());
        while ((int)tr.trail.size() > trail_len_) tr.trail.pop_front();
      }

      RCLCPP_INFO(get_logger(),
        "[EKF] Updated Track %d | pos=(%.2f, %.2f, %.2f) vel=(%.2f, %.2f, %.2f) d2=%.4f",
        tr.id, tr.x(0), tr.x(1), tr.x(2), tr.x(3), tr.x(4), tr.x(5), best_d2);

      // Log errors vs nearest container GT + aquabot range & range error
      logErrors(stamp, tr);

    } else {
      // Create new track
      createTrack(z, stamp, msg->class_id, msg->drone_ns);
    }
  }

  void createTrack(const Eigen::Vector3d &z_world, const rclcpp::Time &stamp, int class_id, const std::string &ns)
  {
    ObstacleTrack tr;
    tr.id = next_track_id_++;
    tr.class_id = class_id;
    tr.drone_ns = ns;
    tr.x.setZero();
    tr.x.head<3>() = z_world;
    tr.P.setZero();
    tr.P.topLeftCorner<3,3>()     = init_var_pos_ * Eigen::Matrix3d::Identity();
    tr.P.bottomRightCorner<3,3>() = init_var_vel_ * Eigen::Matrix3d::Identity();
    tr.last_update = stamp;
    if (draw_trail_) tr.trail.push_back(tr.x.head<3>());
    tracks_[tr.id] = tr;

    RCLCPP_INFO(get_logger(), "[EKF] Created Track %d at (%.2f, %.2f, %.2f) cls=%d ns=%s",
                tr.id, z_world(0), z_world(1), z_world(2), class_id, ns.c_str());
  }

  // Compute nearest container GT, print & CSV:
  //  - errors (est - GT)
  //  - Aquabot↔Tracked range (estimate)
  //  - Aquabot↔GT-container range (truth)
  //  - Range error (estimate - truth)
  void logErrors(const rclcpp::Time& stamp, const ObstacleTrack& tr)
  {
    // nearest containerN (for GT anchor)
    std::string best_name = "unmatched";
    double best_dx=0, best_dy=0, best_dz=0, best_dist=std::numeric_limits<double>::infinity();
    double gt_x=std::numeric_limits<double>::quiet_NaN();
    double gt_y=std::numeric_limits<double>::quiet_NaN();
    double gt_z=std::numeric_limits<double>::quiet_NaN();

    for (const auto& kv : gt_) {
      if (kv.first.rfind("container", 0) != 0) continue; // only containers
      const auto& gp = kv.second;
      double dx = tr.x(0) - gp[0];
      double dy = tr.x(1) - gp[1];
      double dz = tr.x(2) - gp[2];
      double dd = std::sqrt(dx*dx + dy*dy + dz*dz);
      if (dd < best_dist) { best_dist=dd; best_dx=dx; best_dy=dy; best_dz=dz;
                            best_name=kv.first; gt_x=gp[0]; gt_y=gp[1]; gt_z=gp[2]; }
    }

    // Aquabot pose
    bool have_aq = false;
    double ax=0, ay=0, az=0;
    auto itA = gt_.find("aquabot");
    if (itA != gt_.end()) {
      ax = itA->second[0]; ay = itA->second[1]; az = itA->second[2];
      have_aq = true;
    }

    // Ranges
    double aq_range_est = std::numeric_limits<double>::quiet_NaN();
    double aq_range_gt  = std::numeric_limits<double>::quiet_NaN();
    double aq_range_err = std::numeric_limits<double>::quiet_NaN();

    if (have_aq) {
      // Aquabot ↔ Tracked (estimate)
      double dx_e = tr.x(0) - ax, dy_e = tr.x(1) - ay, dz_e = tr.x(2) - az;
      aq_range_est = std::sqrt(dx_e*dx_e + dy_e*dy_e + dz_e*dz_e);

      // Aquabot ↔ Container GT (truth) if we matched a GT container
      if (best_name != "unmatched") {
        double dx_g = gt_x - ax, dy_g = gt_y - ay, dz_g = gt_z - az;
        aq_range_gt = std::sqrt(dx_g*dx_g + dy_g*dy_g + dz_g*dz_g);
        aq_range_err = aq_range_est - aq_range_gt; // signed error
      }
    }

    // Console logs – concise & readable
    if (best_name != "unmatched") {
      RCLCPP_INFO(get_logger(),
        "trk %d cls %d | GT=%s  err=(%.2f,%.2f,%.2f)|d|=%.2f  AqRange est=%.2f gt=%.2f err=%.2f",
        tr.id, tr.class_id, best_name.c_str(),
        best_dx, best_dy, best_dz, best_dist,
        aq_range_est, aq_range_gt, aq_range_err);
    } else {
      RCLCPP_INFO(get_logger(),
        "trk %d cls %d | GT=unmatched  AqRange est=%.2f",
        tr.id, tr.class_id, aq_range_est);
    }

    // CSV
    if (csv_) {
      csv_ << std::fixed << std::setprecision(6)
           << stamp.seconds() << ","
           << tr.id << "," << tr.class_id << "," << tr.drone_ns << ","
           << tr.x(0) << "," << tr.x(1) << "," << tr.x(2) << ","
           << best_name << "," << gt_x << "," << gt_y << "," << gt_z << ","
           << best_dx << "," << best_dy << "," << best_dz << "," << best_dist << ","
           << aq_range_est << "," << aq_range_gt << "," << aq_range_err
           << "\n";
      csv_.flush();
    }

    // Publish compact errors array [dx,dy,dz,‖Δ‖]
    std_msgs::msg::Float32MultiArray arr;
    arr.data = { (float)best_dx, (float)best_dy, (float)best_dz, (float)best_dist };
    errors_pub_->publish(arr);

    // Report line
    std_msgs::msg::String rep;
    std::ostringstream os;
    os << "trk " << tr.id << " cls " << tr.class_id
       << " GT=" << best_name << " err=("
       << std::setprecision(3) << best_dx << "," << best_dy << "," << best_dz
       << ") |d|=" << best_dist
       << "  AqRange est=" << aq_range_est << " gt=" << aq_range_gt
       << " err=" << aq_range_err;
    rep.data = os.str();
    report_pub_->publish(rep);
  }

  void onTimer()
  {
    pruneOldTracks();
    publishTrackedObstacles();
  }

  void pruneOldTracks()
  {
    const rclcpp::Time t = now();
    std::vector<int> drop;
    for (auto &kv : tracks_) {
      if ((t - kv.second.last_update).seconds() > track_prune_time_) drop.push_back(kv.first);
    }
    for (int id : drop) {
      tracks_.erase(id);
      RCLCPP_INFO(get_logger(), "[EKF] Pruned Track %d", id);
    }
  }

  // ---- Covariance viz helpers ----
  static void eigSym3(const Eigen::Matrix3d& S, Eigen::Vector3d& eval, Eigen::Matrix3d& evec)
  {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(S);
    eval = es.eigenvalues();   // ascending
    evec = es.eigenvectors();  // columns = eigenvectors
  }
  static geometry_msgs::msg::Point toPoint(const Eigen::Vector3d& v) {
    geometry_msgs::msg::Point p; p.x=v(0); p.y=v(1); p.z=v(2); return p;
  }

  void publishTrackedObstacles()
  {
    geometry_msgs::msg::PoseArray pa;
    pa.header.stamp = now();
    pa.header.frame_id = "world";

    visualization_msgs::msg::MarkerArray mks;
    TrackedObstacleArray out;
    out.header = pa.header;

    int id = 0;
    for (const auto &kv : tracks_) {
      const auto &tr = kv.second;

      // Pose
      geometry_msgs::msg::Pose pose;
      pose.position.x = tr.x(0); pose.position.y = tr.x(1); pose.position.z = tr.x(2);
      pose.orientation.w = 1.0;
      pa.poses.push_back(pose);

      // TrackedObstacle (3x3 pos covariance)
      TrackedObstacle tobs;
      tobs.header = out.header;
      tobs.id = tr.id;
      tobs.class_id = tr.class_id;
      tobs.drone_ns = tr.drone_ns;
      tobs.position.x = tr.x(0); tobs.position.y = tr.x(1); tobs.position.z = tr.x(2);
      Eigen::Matrix3d Ppos = tr.P.topLeftCorner<3,3>();
      for (int r=0;r<3;++r) for (int c=0;c<3;++c) tobs.covariance[r*3+c] = Ppos(r,c);
      out.obstacles.push_back(tobs);

      // Color
      RGB col = colorByClassAndDrone(tr.class_id, tr.drone_ns);

      // Sphere
      {
        visualization_msgs::msg::Marker mk;
        mk.header = out.header;
        mk.ns = "trk_sphere";
        mk.id = id++;
        mk.type = visualization_msgs::msg::Marker::SPHERE;
        mk.action = visualization_msgs::msg::Marker::ADD;
        mk.pose = pose;
        mk.scale.x = sphere_scale_; mk.scale.y = sphere_scale_; mk.scale.z = sphere_scale_;
        mk.color.a = 1.0; mk.color.r = col.r; mk.color.g = col.g; mk.color.b = col.b;
        mk.lifetime = rclcpp::Duration::from_seconds(marker_life_s_);
        mks.markers.push_back(mk);
      }

      // Trail
      if (draw_trail_ && tr.trail.size() >= 2) {
        visualization_msgs::msg::Marker ln;
        ln.header = out.header;
        ln.ns = "trk_trail";
        ln.id = id++;
        ln.type = visualization_msgs::msg::Marker::LINE_STRIP;
        ln.action = visualization_msgs::msg::Marker::ADD;
        ln.scale.x = 0.04;
        ln.color.a = 0.9; ln.color.r = col.r; ln.color.g = col.g; ln.color.b = col.b;
        ln.lifetime = rclcpp::Duration::from_seconds(marker_life_s_);
        for (const auto& v : tr.trail) ln.points.push_back(toPoint(v));
        mks.markers.push_back(ln);
      }

      // Covariance principal axes
      if (show_cov_axes_) {
        Eigen::Vector3d eval; Eigen::Matrix3d evec;
        eigSym3(Ppos, eval, evec);
        for (int i=0;i<3;++i) eval(i) = std::max(eval(i), 0.0);
        double s = sigma_axes_;
        Eigen::Vector3d a = s*std::sqrt(eval(0)) * evec.col(0);
        Eigen::Vector3d b = s*std::sqrt(eval(1)) * evec.col(1);
        Eigen::Vector3d c = s*std::sqrt(eval(2)) * evec.col(2);

        visualization_msgs::msg::Marker ax;
        ax.header = out.header;
        ax.ns = "trk_cov_axes";
        ax.id = id++;
        ax.type = visualization_msgs::msg::Marker::LINE_LIST;
        ax.action = visualization_msgs::msg::Marker::ADD;
        ax.scale.x = axes_width_;
        ax.color.a = 0.9; ax.color.r = 0.0; ax.color.g = 0.0; ax.color.b = 0.0;
        ax.lifetime = rclcpp::Duration::from_seconds(marker_life_s_);

        Eigen::Vector3d p0 = tr.x.head<3>();
        ax.points.push_back(toPoint(p0 - a)); ax.points.push_back(toPoint(p0 + a));
        ax.points.push_back(toPoint(p0 - b)); ax.points.push_back(toPoint(p0 + b));
        ax.points.push_back(toPoint(p0 - c)); ax.points.push_back(toPoint(p0 + c));
        mks.markers.push_back(ax);
      }

      // XY covariance ellipse
      if (show_cov_xy_ellipse_) {
        Eigen::Matrix2d Pxy = Ppos.topLeftCorner<2,2>();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(Pxy);
        Eigen::Vector2d eval = es.eigenvalues().cwiseMax(0.0);
        Eigen::Matrix2d evec = es.eigenvectors();

        double s = sigma_ellipse_;
        double a = s*std::sqrt(eval(1));
        double b = s*std::sqrt(eval(0));

        std::vector<geometry_msgs::msg::Point> pts; pts.reserve(ellipse_points_+1);
        for (int i=0;i<=ellipse_points_; ++i) {
          double th = (2.0*M_PI*i)/ellipse_points_;
          Eigen::Vector2d q(a*std::cos(th), b*std::sin(th));
          Eigen::Vector2d w = evec * q;
          geometry_msgs::msg::Point gp;
          gp.x = tr.x(0) + w(0);
          gp.y = tr.x(1) + w(1);
          gp.z = tr.x(2);
          pts.push_back(gp);
        }

        visualization_msgs::msg::Marker el;
        el.header = out.header;
        el.ns = "trk_cov_xy";
        el.id = id++;
        el.type = visualization_msgs::msg::Marker::LINE_STRIP;
        el.action = visualization_msgs::msg::Marker::ADD;
        el.scale.x = ellipse_width_;
        el.color.a = 0.9; el.color.r = col.r; el.color.g = col.g; el.color.b = col.b;
        el.lifetime = rclcpp::Duration::from_seconds(marker_life_s_);
        el.points = std::move(pts);
        mks.markers.push_back(el);
      }

      // Text label
      {
        visualization_msgs::msg::Marker lab;
        lab.header = out.header;
        lab.ns = "trk_text";
        lab.id = id++;
        lab.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        lab.action = visualization_msgs::msg::Marker::ADD;
        lab.pose = pose;
        lab.pose.position.z += (sphere_scale_ * 0.9);
        lab.scale.z = label_scale_;
        lab.color.a = 1.0;
        lab.color.r = 0.0; lab.color.g = 0.0; lab.color.b = 0.0;
        lab.lifetime = rclcpp::Duration::from_seconds(marker_life_s_);
        char buf[160];
        std::snprintf(buf, sizeof(buf), "trk:%d cls:%d", tr.id, tr.class_id);
        lab.text = buf;
        mks.markers.push_back(lab);
      }
    }

    pose_pub_->publish(pa);
    marker_pub_->publish(mks);
    tracked_pub_->publish(out);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiObstacleTrackerEKF>());
  rclcpp::shutdown();
  return 0;
}
