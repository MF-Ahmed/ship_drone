// StereoObstacleLocalizer.cpp
// Build in your crazyflie_yolo (or target) package
//
// World-frame output, z=0 plane, no stereo.  Key features:
//  - Correct optical->camera_link rotation (REP 103)
//  - TF fallback when extrapolation occurs
//  - Use bbox bottom-center; optional multi-sample averaging along bottom
//  - Print YOLO class + GT/DET positions and error in logs
//  - CSV includes GT xyz + error components + norm + Aquabot pose
//  - CSV now also includes Aquabot–detection range, Aquabot–GT range, and range error
//  - RViz markers show GT distance + Aquabot range info
//  - Fixed ROS 2 message init style (no brace-init for Vector3)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <array>
#include <iomanip>
#include <sstream>
#include <limits>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <vector>

#include "crazyflie_yolo/msg/detection3_d_stamped.hpp"

using crazyflie_yolo::msg::Detection3DStamped;
using std::placeholders::_1;
using std::placeholders::_2;

class StereoObstacleLocalizer : public rclcpp::Node {
public:
  StereoObstacleLocalizer()
  : Node("stereo_obstacle_localizer"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) {

    using namespace message_filters;

    namespace_ = this->get_namespace(); // e.g. "/drone1"
    RCLCPP_INFO(this->get_logger(), "Node NS: %s", namespace_.c_str());

    // Parameters
    gate_m_   = this->declare_parameter<double>("gate_m", 100.0); // max distance for valid match
    ground_z_ = this->declare_parameter<double>("ground_z", 0.0); // projection plane

    log_dir_  = this->declare_parameter<std::string>(
                  "log_dir",
                  "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs");
                  
    multi_sample_bottom_ = this->declare_parameter<bool>("multi_sample_bottom", true);
    min_score_           = this->declare_parameter<double>("min_score", 0.50);
    gate_frac_           = this->declare_parameter<double>("gate_frac", 0.04);
    gate_min_            = this->declare_parameter<double>("gate_min", 6.0);

    // Optional map of class index -> human-readable name (comma-separated)
    auto labels_csv = this->declare_parameter<std::string>(
      "class_labels", "container1,container2,container3,container4,container5");
    {
      std::stringstream ss(labels_csv);
      for (std::string tok; std::getline(ss, tok, ','); ) {
        tok.erase(std::remove_if(tok.begin(), tok.end(), ::isspace), tok.end());
        if (!tok.empty()) class_labels_.push_back(tok);
      }
    }

    // Subscribers (sync left image + YOLO dets)
    left_sub_.subscribe(this, "downward_left_camera/image_raw");
    yolo_sub_.subscribe(this, "yolo_detections");
    sync_ = std::make_shared<Synchronizer<Sync2>>(Sync2(20)); // smaller queue
    sync_->connectInput(left_sub_, yolo_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.15));
    sync_->registerCallback(std::bind(&StereoObstacleLocalizer::callback, this, _1, _2));

    // Camera info (left)
    left_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "downward_left_camera/camera_info", rclcpp::SensorDataQoS(),
      std::bind(&StereoObstacleLocalizer::leftInfoCallback, this, _1));

    // Ground-truth odometry subscriptions (world frame)
    auto qos_sensor = rclcpp::SensorDataQoS();
    gt_subs_.push_back(
      this->create_subscription<nav_msgs::msg::Odometry>(
        "/aquabot/odometry", qos_sensor,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { this->gtOdomCb(msg, "aquabot"); }
      )
    );
    for (int i = 1; i <= 5; ++i) {
      std::string name = "container" + std::to_string(i);
      gt_subs_.push_back(
        this->create_subscription<nav_msgs::msg::Odometry>(
          "/" + name + "/odometry", qos_sensor,
          [this, name](nav_msgs::msg::Odometry::ConstSharedPtr msg) { this->gtOdomCb(msg, name); }
        )
      );
    }

    // Publishers
    detection_pub_ = this->create_publisher<Detection3DStamped>("detections_3d", 20);
    marker_pub_    = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", 10);
    errors_pub_    = this->create_publisher<std_msgs::msg::Float32MultiArray>("/gt_eval/errors", 10);
    report_pub_    = this->create_publisher<std_msgs::msg::String>("/gt_eval/report", 10);

    openCsvLogger();

    RCLCPP_INFO(this->get_logger(),
      "z-plane=%.2f, gate_m=%.1f, gate_frac=%.3f min=%.1f, multi_bottom=%s, min_score=%.2f",
      ground_z_, gate_m_, gate_frac_, gate_min_, multi_sample_bottom_?"true":"false", min_score_);
  }

  ~StereoObstacleLocalizer() override {
    if (csv_.is_open()) csv_.close();
  }

private:
  // message_filters policy (2 inputs)
  using Sync2 = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, vision_msgs::msg::Detection2DArray>;

  message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> yolo_sub_;
  std::shared_ptr<message_filters::Synchronizer<Sync2>> sync_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
  rclcpp::Publisher<Detection3DStamped>::SharedPtr              detection_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr     errors_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                report_pub_;

  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> gt_subs_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Intrinsics (left)
  cv::Mat K_, D_;
  bool have_cam_ = false;
  int img_w_ = 0, img_h_ = 0;

  // Ground truth store: name -> (x,y,z)
  std::unordered_map<std::string, std::array<double,3>> gt_;
  double gate_m_{100.0};
  double ground_z_{0.0};

  // params
  std::string log_dir_;
  bool   multi_sample_bottom_{true};
  double min_score_{0.5};
  double gate_frac_{0.04};
  double gate_min_{6.0};

  // Markers
  visualization_msgs::msg::MarkerArray marker_array_;
  int next_marker_id_ = 0;

  // CSV
  std::ofstream csv_;
  std::string namespace_;

  // Optional class label list (index-aligned with YOLO id if you want to supply one)
  std::vector<std::string> class_labels_;

  struct RGB { double r,g,b; };
  static inline const std::array<RGB,5> kClassPalette {{
    {1.0, 0.0, 0.0},  // c1: bright red
    {0.0, 0.8, 0.8},  // c2: cyan
    {0.8, 0.0, 0.8},  // c3: magenta
    {1.0, 1.0, 0.0},  // c4: yellow
    {0.0, 1.0, 0.0}   // c5: bright green
  }};

  static inline RGB colorForClass(int cls){
    if (cls>=0 && cls<(int)kClassPalette.size()) return kClassPalette[cls];
    return {0.6,0.6,0.6};
  }

  static bool startsWith(const std::string& s, const char* p) { return s.rfind(p, 0) == 0; }

  static int containerIndex0(const std::string& cls_str) {
    if (startsWith(cls_str, "container") && cls_str.size() > 9) {
      try { int n = std::stoi(cls_str.substr(9)); return n - 1; } catch (...) {}
    }
    return -1;
  }

  static std::string shortenClass(const std::string& cls_str) {
    if (startsWith(cls_str, "container") && cls_str.size() > 9) return "c" + cls_str.substr(9);
    return cls_str;
  }

  // CameraInfo
  void leftInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    K_ = cv::Mat(3,3,CV_64F,(void*)msg->k.data()).clone();
    D_ = cv::Mat(1,(int)msg->d.size(),CV_64F,(void*)msg->d.data()).clone();
    img_w_ = (int)msg->width; img_h_ = (int)msg->height;
    have_cam_ = true;
    RCLCPP_INFO_ONCE(this->get_logger(), "Left CameraInfo received (%dx%d).", img_w_, img_h_);
  }

  // CSV path + header
  void openCsvLogger() {
    std::filesystem::path base{log_dir_};
    std::error_code ec;
    std::filesystem::create_directories(base, ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "create_directories('%s') failed: %s",
                   base.string().c_str(), ec.message().c_str());
      return;
    }
    std::string ns = namespace_;
    if (!ns.empty() && ns.front()=='/') ns.erase(0,1);
    if (ns.empty()) ns = "root";

    std::time_t t = std::time(nullptr);
    std::tm tm{}; localtime_r(&t,&tm);
    std::ostringstream fname;
    fname << "detections_" << ns << "_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
    const std::filesystem::path path = base / fname.str();

    csv_.open(path, std::ios::out);
    if (!csv_) {
      RCLCPP_ERROR(this->get_logger(), "Could not open CSV at %s", path.string().c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Logging CSV to: %s", path.string().c_str());
      csv_ << "stamp,ns,class,score,u,v,"
           << "world_x,world_y,world_z,"
           << "gt_name,gt_x,gt_y,gt_z,"
           << "err_x,err_y,err_z,err_norm,"
           << "aquabot_x,aquabot_y,aquabot_z,"
           << "aq_det_range,aq_gt_range,aq_range_err\n";
      csv_.flush();
    }
  }

  // Pixel -> normalized ray in camera *optical* frame (undistort single point)
  bool pixelToRayCam(double u, double v, cv::Vec3d& dir_cam) {
    if (!have_cam_) return false;
    std::vector<cv::Point2f> src(1), dst(1);
    src[0] = cv::Point2f((float)u,(float)v);
    cv::Mat K32; K_.convertTo(K32, CV_32F);
    cv::Mat D32; D_.convertTo(D32, CV_32F);
    cv::undistortPoints(src, dst, K32, D32);  // normalized optical coords
    dir_cam = cv::Vec3d(dst[0].x, dst[0].y, 1.0);
    double n = std::sqrt(dir_cam[0]*dir_cam[0]+dir_cam[1]*dir_cam[1]+dir_cam[2]*dir_cam[2]);
    if (n <= 1e-9) return false;
    dir_cam /= n;
    return true;
  }

  // Intersect ray with z = ground_z_
  bool intersectRayWithZPlane(const geometry_msgs::msg::Point& Cw,
                              const geometry_msgs::msg::Vector3& Dw,
                              geometry_msgs::msg::Point& Pw) {
    const double eps = 1e-9;
    if (std::abs(Dw.z) < eps) return false;
    double t = (ground_z_ - Cw.z) / Dw.z;
    if (t <= 0.0) return false; // forward only
    Pw.x = Cw.x + t*Dw.x;
    Pw.y = Cw.y + t*Dw.y;
    Pw.z = ground_z_;
    return true;
  }

  // Ground-truth odom callback
  void gtOdomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg, const std::string& name) {
    const auto& p = msg->pose.pose.position;
    gt_[name] = {p.x, p.y, p.z};
  }

  void callback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
                const vision_msgs::msg::Detection2DArray::ConstSharedPtr& dets_msg) {

    if (!have_cam_) return;

    // Frames and time
    std::string ns = namespace_;
    if (!ns.empty() && ns.front()=='/') ns.erase(0,1);
    const std::string cam_frame = ns + "/downward_left_camera_link";
    const rclcpp::Time stamp = img_msg->header.stamp;

    // Camera pose in world with fallback
    geometry_msgs::msg::TransformStamped Twc;
    try {
      Twc = tf_buffer_.lookupTransform("world", cam_frame, stamp, tf2::durationFromSec(0.2));
    } catch (const tf2::ExtrapolationException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF world <- %s at stamp %.3f failed (%s). Using latest TF.",
                           cam_frame.c_str(), stamp.seconds(), ex.what());
      Twc = tf_buffer_.lookupTransform("world", cam_frame, rclcpp::Time(0));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF world <- %s failed: %s", cam_frame.c_str(), ex.what());
      return;
    }

    geometry_msgs::msg::Point Cw;
    Cw.x = Twc.transform.translation.x;
    Cw.y = Twc.transform.translation.y;
    Cw.z = Twc.transform.translation.z;

    tf2::Quaternion q(Twc.transform.rotation.x, Twc.transform.rotation.y,
                      Twc.transform.rotation.z, Twc.transform.rotation.w);
    tf2::Matrix3x3 R(q); // camera_link -> world

    // Also fetch Aquabot position in world at this stamp (for CSV / range error)
    double aq_x = std::numeric_limits<double>::quiet_NaN();
    double aq_y = std::numeric_limits<double>::quiet_NaN();
    double aq_z = std::numeric_limits<double>::quiet_NaN();
    try {
      auto Twa = tf_buffer_.lookupTransform("world", "aquabot/base_link",
                                            stamp, tf2::durationFromSec(0.2));
      aq_x = Twa.transform.translation.x;
      aq_y = Twa.transform.translation.y;
      aq_z = Twa.transform.translation.z;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF world <- aquabot/base_link at %.3f failed (%s).", stamp.seconds(), ex.what());
      try {
        auto Twa = tf_buffer_.lookupTransform("world", "aquabot/base_link", rclcpp::Time(0));
        aq_x = Twa.transform.translation.x;
        aq_y = Twa.transform.translation.y;
        aq_z = Twa.transform.translation.z;
      } catch (...) {}
    }

    visualization_msgs::msg::MarkerArray new_markers;
    std_msgs::msg::Float32MultiArray errors;
    std_msgs::msg::String report;
    std::ostringstream rep;

    if (gt_.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "No ground-truth odometry yet; computing 3D points but not errors.");
    }

    for (const auto& det : dets_msg->detections) {
      // Use bottom-center of bbox (better for ground contact)
      double u = det.bbox.center.position.x;
      double v = det.bbox.center.position.y + 0.5 * det.bbox.size_y;

      // Optional multi-sample along bottom edge
      geometry_msgs::msg::Point Pw;
      bool have_pw=false;
      if (multi_sample_bottom_) {
        std::array<std::pair<double,double>,5> edge {{
          {u - 0.4*det.bbox.size_x, v},
          {u - 0.2*det.bbox.size_x, v},
          {u,                      v},
          {u + 0.2*det.bbox.size_x, v},
          {u + 0.4*det.bbox.size_x, v},
        }};
        int ok=0; double sx=0, sy=0;
        for (auto [uu, vv] : edge) {
          cv::Vec3d dir_cam;
          if (!pixelToRayCam(uu, vv, dir_cam)) continue;
          // optical -> camera_link rotation
          tf2::Matrix3x3 R_opt_to_cam(0,0,1, -1,0,0, 0,-1,0);
          tf2::Vector3 d_opt(dir_cam[0], dir_cam[1], dir_cam[2]);
          tf2::Vector3 d_cam = R_opt_to_cam * d_opt;
          tf2::Vector3 d_world = R * d_cam;

          geometry_msgs::msg::Vector3 Dw;
          Dw.x = d_world.x(); Dw.y = d_world.y(); Dw.z = d_world.z();
          geometry_msgs::msg::Point   Pwi;
          if (!intersectRayWithZPlane(Cw, Dw, Pwi)) continue;
          sx += Pwi.x; sy += Pwi.y; ok++;
        }
        if (ok > 0) {
          Pw.x = sx/ok; Pw.y = sy/ok; Pw.z = ground_z_;
          have_pw = true;
        }
      }
      if (!have_pw) { // fallback: single sample
        cv::Vec3d dir_cam;
        if (!pixelToRayCam(u, v, dir_cam)) continue;
        tf2::Matrix3x3 R_opt_to_cam(0,0,1, -1,0,0, 0,-1,0);
        tf2::Vector3 d_opt(dir_cam[0], dir_cam[1], dir_cam[2]);
        tf2::Vector3 d_cam = R_opt_to_cam * d_opt;
        tf2::Vector3 d_world = R * d_cam;
        geometry_msgs::msg::Vector3 Dw;
        Dw.x = d_world.x(); Dw.y = d_world.y(); Dw.z = d_world.z();
        if (!intersectRayWithZPlane(Cw, Dw, Pw)) continue;
      }

      // CLASS / SCORE EXTRACTION
      int class_id_out = -1;
      float score = 0.0f;
      std::string cls_str = "?";
      std::string class_name_out = "?";
      int cls_color_id = -1;

      if (!det.results.empty()) {
        cls_str = det.results[0].hypothesis.class_id;  // string (could be "0" or "container1")
        score   = det.results[0].hypothesis.score;

        // Try to parse numeric class id, then map to a friendly name if provided
        try {
          int idx = std::stoi(cls_str);
          class_id_out = idx; // numeric YOLO id
          class_name_out = (idx >= 0 && idx < (int)class_labels_.size())
                            ? class_labels_[idx]
                            : cls_str; // fallback to raw string if no label provided
        } catch (...) {
          // non-numeric id; publish string as-is
          class_name_out = cls_str;
        }

        // For viz colors: interpret names like "container3" -> 0-based index
        cls_color_id = containerIndex0(class_name_out);
      }

      if (score < (float)min_score_) continue; // gate by confidence

      // NEAREST GT MATCH
      std::string best_name = "unmatched";
      double best_dx=0, best_dy=0, best_dz=0, best_dist=std::numeric_limits<double>::infinity();
      double gt_x=std::numeric_limits<double>::quiet_NaN();
      double gt_y=std::numeric_limits<double>::quiet_NaN();
      double gt_z=std::numeric_limits<double>::quiet_NaN();

      if (!gt_.empty()) {
        for (const auto& kv : gt_) {
          const auto& gp = kv.second; // {x,y,z}
          double dx = Pw.x - gp[0];
          double dy = Pw.y - gp[1];
          double dz = Pw.z - gp[2];
          double dd = std::sqrt(dx*dx + dy*dy + dz*dz);
          if (dd < best_dist) {
            best_dist = dd; best_dx = dx; best_dy = dy; best_dz = dz;
            best_name = kv.first; gt_x = gp[0]; gt_y = gp[1]; gt_z = gp[2];
          }
        }
        // Range-dependent gate
        if (gate_frac_ > 0.0) {
          double range = std::hypot(Pw.x, Pw.y);
          double gate = std::max(gate_min_, gate_frac_ * range);
          if (best_dist > gate) best_name = "unmatched";
        } else if (best_dist > gate_m_) {
          best_name = "unmatched";
        }
      }

      // AQUABOT RANGE & RANGE ERROR
      double aq_det_range = std::numeric_limits<double>::quiet_NaN();
      double aq_gt_range  = std::numeric_limits<double>::quiet_NaN();
      double aq_range_err = std::numeric_limits<double>::quiet_NaN();

      // Only if we have Aquabot pose
      if (!std::isnan(aq_x) && !std::isnan(aq_y) && !std::isnan(aq_z)) {
        // Aquabot ↔ detected container (estimate)
        double dx_e = Pw.x - aq_x;
        double dy_e = Pw.y - aq_y;
        double dz_e = Pw.z - aq_z;
        aq_det_range = std::sqrt(dx_e*dx_e + dy_e*dy_e + dz_e*dz_e);

        // Aquabot ↔ GT container (truth), only if matched
        if (best_name != "unmatched" &&
            !std::isnan(gt_x) && !std::isnan(gt_y) && !std::isnan(gt_z)) {
          double dx_g = gt_x - aq_x;
          double dy_g = gt_y - aq_y;
          double dz_g = gt_z - aq_z;
          aq_gt_range = std::sqrt(dx_g*dx_g + dy_g*dy_g + dz_g*dz_g);
          aq_range_err = aq_det_range - aq_gt_range;  // signed error
        }
      }

      // PUBLISH 3D DETECTION
      Detection3DStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = "world";
      msg.position.x = Pw.x; msg.position.y = Pw.y; msg.position.z = Pw.z;
      msg.class_id   = class_id_out;
      msg.class_name = class_name_out;
      msg.score = score;
      std::string ns_clean = namespace_;
      if (!ns_clean.empty() && ns_clean.front()=='/') ns_clean.erase(0,1);
      msg.drone_ns = ns_clean;
      detection_pub_->publish(msg);

      // CSV LOGGING
      if (csv_) {
        csv_ << std::fixed << std::setprecision(6)
             << stamp.seconds() << "," << ns_clean << ","
             << class_id_out << "," << score << ","
             << u << "," << v << ","
             << Pw.x << "," << Pw.y << "," << Pw.z << ","
             << best_name << ","
             << gt_x << "," << gt_y << "," << gt_z << ","
             << best_dx << "," << best_dy << "," << best_dz << "," << best_dist << ","
             << aq_x << "," << aq_y << "," << aq_z << ","
             << aq_det_range << "," << aq_gt_range << "," << aq_range_err
             << "\n";
      }

      // MARKERS (short label)
      RGB col = colorForClass(cls_color_id);
      int id_base = next_marker_id_; next_marker_id_ += 2;

      visualization_msgs::msg::Marker sph;
      sph.header.frame_id = "world"; sph.header.stamp = stamp;
      sph.ns = ns_clean + "_obstacles"; sph.id = id_base;
      sph.type = visualization_msgs::msg::Marker::SPHERE;
      sph.action = visualization_msgs::msg::Marker::ADD;
      sph.pose.position = Pw; sph.pose.orientation.w = 1.0;
      sph.scale.x = sph.scale.y = sph.scale.z = 0.45;
      sph.color.a = 1.0; sph.color.r = col.r; sph.color.g = col.g; sph.color.b = col.b;
      sph.lifetime = rclcpp::Duration::from_seconds(2.0);
      new_markers.markers.push_back(sph);

    visualization_msgs::msg::Marker txt = sph;
    txt.id = id_base + 1;
    txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

    std::ostringstream label;
    std::string pred_short = shortenClass(class_name_out);

    // 1) First line: drone name
    label << ns_clean << "\n";

    // 2) Predicted class + score
    label << pred_short << " (" << std::fixed << std::setprecision(2) << score << ")";

    // 3) GT match + container-det error
    if (best_name != "unmatched") {
      std::string gt_short = shortenClass(best_name);
      label << "\nGT=" << gt_short;
            //<< " d=" << std::fixed << std::setprecision(2) << best_dist << "m";
    } else {
      label << "\n(no match)";
    }

    // 4) Aquabot range info (if available)
    //if (!std::isnan(aq_det_range)) {
      //label << "\nAqR=" << std::fixed << std::setprecision(2) << aq_det_range << "m";
      //if (!std::isnan(aq_gt_range) && !std::isnan(aq_range_err)) {
        //label << " (gt=" << std::fixed << std::setprecision(2) << aq_gt_range
              //<< ", e=" << std::fixed << std::setprecision(2) << aq_range_err << ")";
      //}
    //}

    // Optional coords (still commented)
    // label << "\n(" << std::fixed << std::setprecision(2)
    //       << Pw.x << "," << Pw.y << "," << Pw.z << ")";

    txt.text = label.str();
    txt.pose.position.z += 0.6;
    txt.scale.z = 0.35;
    txt.color.r = txt.color.g = txt.color.b = 1.0;
    txt.color.a = 1.0;   // make sure it's visible
    new_markers.markers.push_back(txt);

      // Errors table row for /gt_eval/errors (dx,dy,dz,dist)
      errors.data.insert(errors.data.end(), {
        (float)best_dx, (float)best_dy, (float)best_dz, (float)best_dist
      });
      std::ostringstream line;
      line << "det: cls=" << class_name_out << " score=" << std::fixed << std::setprecision(2) << score
           << " world=(" << std::setprecision(2) << Pw.x << "," << Pw.y << "," << Pw.z << ") "
           << "match=" << best_name << " dist=" << std::setprecision(3) << best_dist << " m";
      rep << line.str() << "\n";
    }

    // Publish markers for this batch
    for (auto& m : new_markers.markers) marker_array_.markers.push_back(m);
    marker_pub_->publish(new_markers);

    // Publish errors array with layout
    if (!errors.data.empty()) {
      std_msgs::msg::Float32MultiArray out = errors;
      const size_t n = out.data.size()/4;
      out.layout.dim.resize(2);
      out.layout.dim[0].label = "detections";
      out.layout.dim[0].size  = (uint32_t)n;
      out.layout.dim[0].stride= (uint32_t)(4*n);
      out.layout.dim[1].label = "metrics";
      out.layout.dim[1].size  = 4;
      out.layout.dim[1].stride= 4;
      errors_pub_->publish(out);
    }

    // Publish report
    if (!rep.str().empty()) {
      std_msgs::msg::String s; s.data = rep.str();
      report_pub_->publish(s);
    }

    if (csv_) csv_.flush();
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoObstacleLocalizer>());
  rclcpp::shutdown();
  return 0;
}
