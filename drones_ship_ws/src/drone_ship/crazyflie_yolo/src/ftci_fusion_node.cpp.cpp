// ftci_fusion_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <unordered_map>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

#include "crazyflie_yolo/msg/tracked_obstacle_array.hpp"
#include "crazyflie_yolo/msg/tracked_obstacle.hpp"

using crazyflie_yolo::msg::TrackedObstacleArray;
using crazyflie_yolo::msg::TrackedObstacle;

struct Track3D {
  int id = -1;
  int class_id = -1;
  std::string class_name;
  std::string drone_ns;

  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Matrix3d P = Eigen::Matrix3d::Identity();
};

static inline double logdetSPD(const Eigen::Matrix3d& P) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(P);
  if (es.info() != Eigen::Success) return std::numeric_limits<double>::quiet_NaN();
  const auto ev = es.eigenvalues();
  double ld = 0.0;
  for (int i=0;i<3;i++) ld += std::log(std::max(1e-12, ev[i]));
  return ld;
}

// Standard CI fuse
static bool fuseCI(const Eigen::Vector3d& x1, const Eigen::Matrix3d& P1,
                   const Eigen::Vector3d& x2, const Eigen::Matrix3d& P2,
                   double omega,
                   Eigen::Vector3d& x_out, Eigen::Matrix3d& P_out)
{
  Eigen::Matrix3d P1inv, P2inv;
  double det1, det2;
  bool ok1, ok2;
  P1.computeInverseAndDetWithCheck(P1inv, det1, ok1);
  P2.computeInverseAndDetWithCheck(P2inv, det2, ok2);
  if (!ok1 || !ok2) return false;

  Eigen::Matrix3d Pinv = (1.0 - omega) * P1inv + omega * P2inv;

  Eigen::Matrix3d P;
  double det;
  bool ok;
  Pinv.computeInverseAndDetWithCheck(P, det, ok);
  if (!ok) return false;

  x_out = P * ((1.0 - omega) * P1inv * x1 + omega * P2inv * x2);
  P_out = P;
  return true;
}

class FtciFusionNode : public rclcpp::Node {
public:
  FtciFusionNode() : Node("ftci_fusion_node")
  {
    input_topics_ = declare_parameter<std::vector<std::string>>(
      "input_topics",
      std::vector<std::string>{
        "/drone1/tracked_obstacles_array",
        "/drone2/tracked_obstacles_array",
        "/drone3/tracked_obstacles_array"
      });

    output_topic_ = declare_parameter<std::string>("output_topic", "/fused_tracked_obstacles_array_ftci");

    assoc_dist_thresh_ = declare_parameter<double>("association_threshold", 8.0);

    // FTCI disagreement threshold for 3D (chi-square)
    // 95%: 7.815, 99%: 11.345
    chi2_thresh_ = declare_parameter<double>("chi2_thresh", 11.345);

    // union inflation scale (extra conservativeness)
    union_scale_ = declare_parameter<double>("union_scale", 1.0);

    // same heuristic omega used in your CI: based on logdet (det-based weighting)
    omega_min_ = declare_parameter<double>("omega_min", 0.05);
    omega_max_ = declare_parameter<double>("omega_max", 0.95);

    fuse_rate_hz_ = declare_parameter<double>("fuse_rate_hz", 10.0);

    pub_ = create_publisher<TrackedObstacleArray>(output_topic_, rclcpp::QoS(10));

    for (const auto& topic : input_topics_) {
      auto sub = create_subscription<TrackedObstacleArray>(
        topic, rclcpp::QoS(20),
        [this, topic](TrackedObstacleArray::ConstSharedPtr msg){
          latest_[topic] = msg;
        });
      subs_.push_back(sub);
      RCLCPP_INFO(get_logger(), "FTCI subscribing: %s", topic.c_str());
    }

    auto period = std::chrono::duration<double>(1.0 / std::max(0.1, fuse_rate_hz_));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                               std::bind(&FtciFusionNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "FTCI fusion publishing: %s", output_topic_.c_str());
  }

private:
  static Eigen::Matrix3d covFromMsg(const TrackedObstacle& o) {
    Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
    // covariance stored as 9-array row-major in your msg
    P(0,0)=o.covariance[0]; P(0,1)=o.covariance[1]; P(0,2)=o.covariance[2];
    P(1,0)=o.covariance[3]; P(1,1)=o.covariance[4]; P(1,2)=o.covariance[5];
    P(2,0)=o.covariance[6]; P(2,1)=o.covariance[7]; P(2,2)=o.covariance[8];
    // fallback if diagonal-only is used by your trackers
    if (P.norm() < 1e-12) {
      P = Eigen::Matrix3d::Identity();
      P(0,0)=o.covariance[0];
      P(1,1)=o.covariance[4];
      P(2,2)=o.covariance[8];
    }
    // ensure SPD-ish
    for (int i=0;i<3;i++) P(i,i)=std::max(P(i,i), 1e-6);
    return P;
  }

  static double omegaFromLogdet(const Eigen::Matrix3d& P1, const Eigen::Matrix3d& P2,
                                double wmin, double wmax)
  {
    const double ld1 = logdetSPD(P1);
    const double ld2 = logdetSPD(P2);
    if (!std::isfinite(ld1) || !std::isfinite(ld2)) return 0.5;
    // smaller logdet => more confident => higher weight
    const double c1 = std::exp(-ld1);
    const double c2 = std::exp(-ld2);
    const double w = (c1 + c2 > 1e-12) ? (c2 / (c1 + c2)) : 0.5;
    return std::min(wmax, std::max(wmin, w));
  }

  bool fuseFTCI(const Track3D& pre, const Track3D& meas, Track3D& out, bool& used_union)
  {
    used_union = false;

    const Eigen::Vector3d d = meas.p - pre.p;
    Eigen::Matrix3d S = pre.P + meas.P;

    Eigen::Matrix3d Sinv;
    double detS;
    bool ok;
    S.computeInverseAndDetWithCheck(Sinv, detS, ok);
    if (!ok) return false;

    const double d2 = d.transpose() * Sinv * d;

    const double omega = omegaFromLogdet(pre.P, meas.P, omega_min_, omega_max_);

    if (d2 <= chi2_thresh_) {
      // normal CI
      Eigen::Vector3d p_out;
      Eigen::Matrix3d P_out;
      if (!fuseCI(pre.p, pre.P, meas.p, meas.P, omega, p_out, P_out)) return false;
      out = pre;
      out.p = p_out;
      out.P = P_out;
      return true;
    }

    // fault-tolerant fallback: keep mean, inflate covariance along discrepancy direction
    used_union = true;
    out = pre;
    const double alpha = union_scale_ * std::max(0.0, (d2 / chi2_thresh_) - 1.0);
    out.P = pre.P + alpha * (d * d.transpose());
    return true;
  }

  int findNearestFused(const std::vector<Track3D>& fused, const Track3D& z) const
  {
    int best = -1;
    double bestd = 1e18;
    for (int i=0;i<(int)fused.size();++i) {
      const double d = (fused[i].p - z.p).norm();
      if (d < bestd) { bestd = d; best = i; }
    }
    if (best >= 0 && bestd <= assoc_dist_thresh_) return best;
    return -1;
  }

  void onTimer()
  {
    // gather latest msgs
    std::vector<TrackedObstacleArray::ConstSharedPtr> msgs;
    msgs.reserve(input_topics_.size());
    rclcpp::Time stamp = now();

    for (const auto& topic : input_topics_) {
      auto it = latest_.find(topic);
      if (it != latest_.end() && it->second) {
        msgs.push_back(it->second);
        stamp = rclcpp::Time(it->second->header.stamp);
      }
    }
    if (msgs.empty()) return;

    // sequential fusion
    std::vector<Track3D> fused;

    // initialize with first message tracks
    for (const auto& o : msgs[0]->obstacles) {
      Track3D tr;
      tr.id = (int)o.id;
      tr.class_id = o.class_id;
      tr.class_name = o.class_name;
      tr.drone_ns = o.drone_ns;
      tr.p = Eigen::Vector3d(o.position.x, o.position.y, o.position.z);
      tr.P = covFromMsg(o);
      fused.push_back(tr);
    }

    // fuse remaining messages
    for (size_t mi=1; mi<msgs.size(); ++mi) {
      for (const auto& o : msgs[mi]->obstacles) {
        Track3D meas;
        meas.id = (int)o.id;
        meas.class_id = o.class_id;
        meas.class_name = o.class_name;
        meas.drone_ns = o.drone_ns;
        meas.p = Eigen::Vector3d(o.position.x, o.position.y, o.position.z);
        meas.P = covFromMsg(o);

        const int idx = findNearestFused(fused, meas);
        if (idx < 0) {
          fused.push_back(meas);
        } else {
          Track3D out;
          bool used_union=false;
          if (fuseFTCI(fused[idx], meas, out, used_union)) {
            fused[idx] = out;
          }
        }
      }
    }

    // publish
    TrackedObstacleArray outmsg;
    outmsg.header.stamp = stamp;
    outmsg.header.frame_id = "world";

    for (const auto& tr : fused) {
      TrackedObstacle o;
      o.id = tr.id;
      o.class_id = tr.class_id;
      o.class_name = tr.class_name;
      o.drone_ns = tr.drone_ns;
      o.position.x = tr.p.x();
      o.position.y = tr.p.y();
      o.position.z = tr.p.z();
      // store covariance row-major
      o.covariance[0]=tr.P(0,0); o.covariance[1]=tr.P(0,1); o.covariance[2]=tr.P(0,2);
      o.covariance[3]=tr.P(1,0); o.covariance[4]=tr.P(1,1); o.covariance[5]=tr.P(1,2);
      o.covariance[6]=tr.P(2,0); o.covariance[7]=tr.P(2,1); o.covariance[8]=tr.P(2,2);
      outmsg.obstacles.push_back(o);
    }

    pub_->publish(outmsg);
  }

private:
  std::vector<std::string> input_topics_;
  std::string output_topic_;

  double assoc_dist_thresh_{8.0};
  double chi2_thresh_{11.345};
  double union_scale_{1.0};
  double omega_min_{0.05};
  double omega_max_{0.95};
  double fuse_rate_hz_{10.0};

  std::unordered_map<std::string, TrackedObstacleArray::ConstSharedPtr> latest_;
  std::vector<rclcpp::Subscription<TrackedObstacleArray>::SharedPtr> subs_;
  rclcpp::Publisher<TrackedObstacleArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FtciFusionNode>());
  rclcpp::shutdown();
  return 0;
}