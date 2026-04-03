// fusion_metrics_node.cpp  (NO NEES; uses Mean logdet(P) only) + TF to world
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>
#include <unordered_map>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <chrono>

#include "crazyflie_yolo/msg/tracked_obstacle_array.hpp"
#include "crazyflie_yolo/msg/tracked_obstacle.hpp"

using crazyflie_yolo::msg::TrackedObstacleArray;
using crazyflie_yolo::msg::TrackedObstacle;

struct Vec3 { double x=0,y=0,z=0; };
static inline double dist3(const Vec3&a,const Vec3&b){
  const double dx=a.x-b.x, dy=a.y-b.y, dz=a.z-b.z;
  return std::sqrt(dx*dx+dy*dy+dz*dz);
}

static Eigen::Matrix3d covFromMsg(const TrackedObstacle& o) {
  Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
  P(0,0)=o.covariance[0]; P(0,1)=o.covariance[1]; P(0,2)=o.covariance[2];
  P(1,0)=o.covariance[3]; P(1,1)=o.covariance[4]; P(1,2)=o.covariance[5];
  P(2,0)=o.covariance[6]; P(2,1)=o.covariance[7]; P(2,2)=o.covariance[8];

  // fallback if diagonal-only is used
  if (P.norm() < 1e-12) {
    P = Eigen::Matrix3d::Identity();
    P(0,0)=o.covariance[0];
    P(1,1)=o.covariance[4];
    P(2,2)=o.covariance[8];
  }
  // ensure not singular
  for (int i=0;i<3;i++) P(i,i)=std::max(P(i,i), 1e-6);
  return P;
}

static inline double logdetSPD(const Eigen::Matrix3d& P) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(P);
  if (es.info() != Eigen::Success) return std::numeric_limits<double>::quiet_NaN();
  const auto ev = es.eigenvalues();
  double ld = 0.0;
  for (int i=0;i<3;i++) ld += std::log(std::max(1e-12, ev[i]));
  return ld;
}

struct MethodAcc {
  long long N = 0;               // matched samples count (cumulative)
  std::vector<double> errors;    // cumulative errors over matched samples
  double logdet_sum = 0.0;
  long long logdet_n = 0;
};

static double percentile(std::vector<double> e, double p){
  if (e.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(e.begin(), e.end());
  const double idx = (p/100.0) * (double)(e.size()-1);
  const size_t i0 = (size_t)std::floor(idx);
  const size_t i1 = std::min(i0+1, e.size()-1);
  const double a = idx - (double)i0;
  return (1.0-a)*e[i0] + a*e[i1];
}

static double rmse(const std::vector<double>& e){
  if (e.empty()) return std::numeric_limits<double>::quiet_NaN();
  long double ss=0.0;
  for (double v: e) ss += (long double)v*(long double)v;
  return std::sqrt((double)(ss/(long double)e.size()));
}

class FusionMetricsNode : public rclcpp::Node {
public:
  FusionMetricsNode() : Node("fusion_metrics_node")
  {
    gt_container_count_ = declare_parameter<int>("gt_container_count", 5);
    dist_thresh_ = declare_parameter<double>("dist_thresh", 8.0);
    eval_rate_hz_ = declare_parameter<double>("eval_rate_hz", 5.0);

    ci_topic_   = declare_parameter<std::string>("ci_topic",   "/fused_tracked_obstacles_array");
    ftci_topic_ = declare_parameter<std::string>("ftci_topic", "/fused_tracked_obstacles_array_ftci");

    csv_path_ = declare_parameter<std::string>("csv_path", "/tmp/fusion_metrics.csv");
    csv_append_ = declare_parameter<bool>("csv_append", false);

    // TF params
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);

    // TF setup
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // GT subs
    for (int i=1;i<=gt_container_count_;++i){
      const std::string topic = "/container"+std::to_string(i)+"/odometry";
      gt_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        topic, rclcpp::SensorDataQoS(),
        [this,i](nav_msgs::msg::Odometry::ConstSharedPtr msg){
          gt_[i] = Vec3{msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
          gt_stamp_ = rclcpp::Time(msg->header.stamp);
        }));
    }

    sub_ci_ = create_subscription<TrackedObstacleArray>(
      ci_topic_, rclcpp::QoS(20),
      [this](TrackedObstacleArray::ConstSharedPtr msg){ latest_ci_ = msg; });

    sub_ftci_ = create_subscription<TrackedObstacleArray>(
      ftci_topic_, rclcpp::QoS(20),
      [this](TrackedObstacleArray::ConstSharedPtr msg){ latest_ftci_ = msg; });

    openCsv();

    auto period = std::chrono::duration<double>(1.0/std::max(0.1, eval_rate_hz_));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                               std::bind(&FusionMetricsNode::tick, this));

    RCLCPP_INFO(get_logger(),
      "FusionMetricsNode | world_frame=%s tf_timeout=%.3f dist_thresh=%.2f",
      world_frame_.c_str(), tf_timeout_sec_, dist_thresh_);
  }

private:
  void openCsv(){
    namespace fs = std::filesystem;
    try{
      fs::path p(csv_path_);
      if (p.has_parent_path()) fs::create_directories(p.parent_path());
    } catch(...) {}

    std::ios::openmode mode = std::ios::out | (csv_append_ ? std::ios::app : std::ios::trunc);
    csv_.open(csv_path_, mode);
    if (!csv_.is_open()){
      RCLCPP_ERROR(get_logger(), "Cannot open CSV: %s", csv_path_.c_str());
      return;
    }
    if (!csv_append_) {
      csv_ << "stamp_sec,method,N,MedErr,RMSE,P95,MeanLogDet\n";
      csv_.flush();
    }
  }

  bool haveGT() const {
    return (int)gt_.size() >= gt_container_count_;
  }

  // Transform point into world_frame_
  bool toWorld(const std::string& src_frame, const rclcpp::Time& stamp, const Vec3& in, Vec3& out) {
    if (src_frame.empty() || src_frame == world_frame_) {
      out = in;
      return true;
    }

    geometry_msgs::msg::PointStamped p;
    p.header.frame_id = src_frame;
    p.header.stamp = stamp;
    p.point.x = in.x; p.point.y = in.y; p.point.z = in.z;

    try {
      auto pw = tf_buffer_->transform(p, world_frame_, tf2::durationFromSec(tf_timeout_sec_));
      out = Vec3{pw.point.x, pw.point.y, pw.point.z};
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF failed %s -> %s : %s", src_frame.c_str(), world_frame_.c_str(), ex.what());
      return false;
    }
  }

  void evalMethod(const std::string& method_name,
                  TrackedObstacleArray::ConstSharedPtr msg,
                  MethodAcc& acc)
  {
    if (!msg) return;
    if (!haveGT()) return;

    const std::string src_frame = msg->header.frame_id;
    const rclcpp::Time stamp(msg->header.stamp);

    std::vector<Vec3> preds_w;
    std::vector<Eigen::Matrix3d> covs;
    preds_w.reserve(msg->obstacles.size());
    covs.reserve(msg->obstacles.size());

    for (const auto& o: msg->obstacles){
      Vec3 p_src{o.position.x,o.position.y,o.position.z};
      Vec3 p_w;
      if (!toWorld(src_frame, stamp, p_src, p_w)) continue;
      preds_w.push_back(p_w);
      covs.push_back(covFromMsg(o));
    }

    if (preds_w.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[%s] no usable preds (frame=%s) after TF", method_name.c_str(), src_frame.c_str());
      return;
    }

    // Greedy nearest GT->pred match (good enough for fusion-level comparison)
    for (int i=1;i<=gt_container_count_;++i){
      const Vec3 g = gt_.at(i);
      double bestd=1e18;
      int best=-1;
      for (int j=0;j<(int)preds_w.size();++j){
        const double d = dist3(g, preds_w[j]);
        if (d<bestd){ bestd=d; best=j; }
      }
      if (best>=0 && bestd <= dist_thresh_){
        acc.N++;
        acc.errors.push_back(bestd);

        const double ld = logdetSPD(covs[best]);
        if (std::isfinite(ld)){
          acc.logdet_sum += ld;
          acc.logdet_n++;
        }
      }
    }
  }

  void writeRow(double stamp_sec, const std::string& method, const MethodAcc& acc)
  {
    if (!csv_.is_open()) return;

    const double med = percentile(acc.errors, 50.0);
    const double r = rmse(acc.errors);
    const double p95 = percentile(acc.errors, 95.0);
    const double mean_ld = (acc.logdet_n>0) ? (acc.logdet_sum/(double)acc.logdet_n)
                                           : std::numeric_limits<double>::quiet_NaN();

    csv_ << stamp_sec << "," << method << "," << acc.N << ","
         << med << "," << r << "," << p95 << ","
         << mean_ld << "\n";
    csv_.flush();
  }

  void tick()
  {
    if (!haveGT()) return;

    const double t = now().seconds();

    evalMethod("ci",   latest_ci_,   ci_acc_);
    evalMethod("ftci", latest_ftci_, ftci_acc_);

    writeRow(t, "ci", ci_acc_);
    writeRow(t, "ftci", ftci_acc_);
  }

private:
  int gt_container_count_{5};
  double dist_thresh_{8.0};
  double eval_rate_hz_{5.0};

  std::string ci_topic_;
  std::string ftci_topic_;
  std::string csv_path_;
  bool csv_append_{false};

  // TF
  std::string world_frame_{"world"};
  double tf_timeout_sec_{0.05};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // GT cache
  std::unordered_map<int, Vec3> gt_;
  rclcpp::Time gt_stamp_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> gt_subs_;

  // Predictions cache
  rclcpp::Subscription<TrackedObstacleArray>::SharedPtr sub_ci_;
  rclcpp::Subscription<TrackedObstacleArray>::SharedPtr sub_ftci_;
  TrackedObstacleArray::ConstSharedPtr latest_ci_;
  TrackedObstacleArray::ConstSharedPtr latest_ftci_;

  MethodAcc ci_acc_;
  MethodAcc ftci_acc_;

  std::ofstream csv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionMetricsNode>());
  rclcpp::shutdown();
  return 0;
}