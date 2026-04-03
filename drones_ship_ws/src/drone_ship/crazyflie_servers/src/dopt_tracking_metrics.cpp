// dopt_tracking_metrics.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <unordered_map>
#include <string>
#include <vector>
#include <fstream>
#include <cmath>
#include <limits>

#include "crazyflie_yolo/msg/tracked_obstacle.hpp"
#include "crazyflie_yolo/msg/tracked_obstacle_array.hpp"

namespace msgs = crazyflie_yolo::msg;

struct TrackState {
  // Measurement session
  bool active = false;
  rclcpp::Time t0;
  double logdet0 = std::numeric_limits<double>::quiet_NaN();

  // Latest
  double logdet = std::numeric_limits<double>::quiet_NaN();
  geometry_msgs::msg::Pose last_assigned_pose; // where drone should hover
  geometry_msgs::msg::Point last_target_pos;
};

class DoptTrackingMetrics : public rclcpp::Node
{
public:
  DoptTrackingMetrics()
  : Node("dopt_tracking_metrics")
  {
    // Params
    drones_ = declare_parameter<std::vector<std::string>>(
      "drones", std::vector<std::string>{"/drone1","/drone2","/drone3"});
    fused_topic_ = declare_parameter<std::string>("fused_topic", "fused_tracked_obstacles_array");
    frame_id_    = declare_parameter<std::string>("frame_id", "world");
    log_csv_path_= declare_parameter<std::string>("log_csv_path", std::string(""));
    thresh_logdet_= declare_parameter<double>("thresh_logdet", 0.02); // done threshold
    text_lifetime_ = declare_parameter<double>("text_lifetime", 0.7);

    // Publishers & subscribers per drone
    for (const auto & ns : drones_) {
      PerDrone pd;
      pd.ns = ns;

      // metrics topics
      pd.pub_logdet = create_publisher<std_msgs::msg::Float64>(ns + "/metrics/logdet", 10);
      pd.pub_delta  = create_publisher<std_msgs::msg::Float64>(ns + "/metrics/delta_logdet", 10);
      pd.pub_rate   = create_publisher<std_msgs::msg::Float64>(ns + "/metrics/delta_rate", 10);

      // listen to assigned pose + obstacle (start of a measurement session)
      pd.sub_pose = create_subscription<geometry_msgs::msg::PoseStamped>(
        ns + "/assigned_pose", 10,
        [this, ns](geometry_msgs::msg::PoseStamped::SharedPtr msg){ onAssignedPose(ns, *msg); });

      pd.sub_obs = create_subscription<msgs::TrackedObstacle>(
        ns + "/assigned_obstacle", 10,
        [this, ns](msgs::TrackedObstacle::SharedPtr msg){ onAssignedObstacle(ns, *msg); });

      per_[ns] = std::move(pd);
    }

    // fused stream
    sub_fused_ = create_subscription<msgs::TrackedObstacleArray>(
      fused_topic_, 10,
      std::bind(&DoptTrackingMetrics::onFused, this, std::placeholders::_1));

    // a single marker publisher
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("dopt_metrics_markers", 10);

    // timer to refresh markers/CSV
    timer_ = create_wall_timer(std::chrono::milliseconds(200),
              std::bind(&DoptTrackingMetrics::onTimer, this));

    RCLCPP_INFO(get_logger(), "dopt_tracking_metrics up; listening to %s", fused_topic_.c_str());
  }

private:
  struct PerDrone {
    std::string ns;

    // live metrics publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_logdet;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_delta;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rate;

    // inputs
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose;
    rclcpp::Subscription<msgs::TrackedObstacle>::SharedPtr           sub_obs;

    // current measurement state keyed by target id
    std::unordered_map<int, TrackState> state;
    // currently assigned target id (latest message wins)
    int current_id = -1;
  };

  std::vector<std::string> drones_;
  std::string fused_topic_;
  std::string frame_id_;
  std::string log_csv_path_;
  double thresh_logdet_{0.02};
  double text_lifetime_{0.7};

  std::unordered_map<std::string, PerDrone> per_;
  rclcpp::Subscription<msgs::TrackedObstacleArray>::SharedPtr sub_fused_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---------- Callbacks ----------

  static double logdet3x3(const std::array<double,9>& C)
  {
    // row-major 3x3
    const double a = C[0], b=C[1], c=C[2];
    const double d = C[3], e=C[4], f=C[5];
    const double g = C[6], h=C[7], i=C[8];
    const double det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
    return std::log(std::max(1e-12, det));
  }

  void onAssignedPose(const std::string& ns, const geometry_msgs::msg::PoseStamped& ps)
  {
    auto & pd = per_.at(ns);
    if (pd.current_id < 0) return; // wait until we know which id is ours

    auto & st = pd.state[pd.current_id];
    st.last_assigned_pose = ps.pose;
    // If we already have a baseline session running, keep it; otherwise start when we see fused.
  }

  void onAssignedObstacle(const std::string& ns, const msgs::TrackedObstacle& tob)
  {
    auto & pd = per_.at(ns);
    pd.current_id = tob.id;

    auto & st = pd.state[tob.id];
    st.last_target_pos = tob.position;

    // If we already got logdet from fused stream, we might initialize st.t0/logdet0 there
    // Here we only keep last geometry and ensure we track this id.
  }

  void onFused(const msgs::TrackedObstacleArray::SharedPtr msg)
  {
    // For each drone’s current_id, see if it’s in this fused message
    for (auto & kv : per_) {
      auto & pd = kv.second;
      const int tid = pd.current_id;
      if (tid < 0) continue;

      for (const auto & ob : msg->obstacles) {
        if (ob.id != tid) continue;

        auto & st = pd.state[tid];
        // compute current logdet
        std::array<double,9> C;
        for (int k=0;k<9;++k) C[k] = ob.covariance[k];
        st.logdet = logdet3x3(C);

        // If session not active, start now (baseline captured)
        if (!st.active) {
          st.active = true;
          st.t0 = now();
          st.logdet0 = st.logdet;
          RCLCPP_INFO(get_logger(), "[%s] start D-opt session id=%d logdet0=%.4g",
                      pd.ns.c_str(), tid, st.logdet0);
        }

        // publish metrics
        publishMetrics(pd.ns, st);
        break;
      }
    }
  }

  void publishMetrics(const std::string& ns, const TrackState& st)
  {
    if (!std::isfinite(st.logdet) || !std::isfinite(st.logdet0)) return;

    std_msgs::msg::Float64 v;
    v.data = st.logdet;
    per_.at(ns).pub_logdet->publish(v);

    std_msgs::msg::Float64 d;
    d.data = st.logdet0 - st.logdet; // improvement
    per_.at(ns).pub_delta->publish(d);

    std_msgs::msg::Float64 r;
    const double dt = (now() - st.t0).seconds();
    r.data = (dt > 1e-3) ? (d.data / dt) : 0.0;
    per_.at(ns).pub_rate->publish(r);

    // Optional: CSV
    if (!log_csv_path_.empty()) {
      std::ofstream f(log_csv_path_, std::ios::app);
      if (f) {
        f << this->now().seconds() << "," << ns << ","
          << d.data << "," << st.logdet << "," << r.data << "," << dt << "\n";
      }
    }
  }

  void onTimer()
  {
    visualization_msgs::msg::MarkerArray arr;
    // clear first
    {
      visualization_msgs::msg::Marker del;
      del.action = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(del);
    }

    int mid = 0;
    for (auto & kv : per_) {
      auto & pd = kv.second;
      const int tid = pd.current_id;
      if (tid < 0) continue;

      auto it = pd.state.find(tid);
      if (it == pd.state.end()) continue;
      const auto & st = it->second;
      if (!st.active || !std::isfinite(st.logdet0) || !std::isfinite(st.logdet)) continue;

      // text marker at target pos
      visualization_msgs::msg::Marker txt;
      txt.header.frame_id = frame_id_;
      txt.header.stamp = now();
      txt.ns = "dopt_metrics";
      txt.id = mid++;
      txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      txt.action = visualization_msgs::msg::Marker::ADD;
      txt.pose.position = st.last_target_pos;
      txt.pose.position.z += 1.0;
      txt.scale.z = 0.35;
      txt.color.a = 1.0; txt.color.r = 1.0; txt.color.g = 1.0; txt.color.b = 1.0;

      const double delta = st.logdet0 - st.logdet;
      const double dt = (now() - st.t0).seconds();
      const double rate = (dt>1e-3)? delta/dt : 0.0;

      std::ostringstream oss;
      oss.setf(std::ios::fixed); oss<<std::setprecision(3);
      oss << kv.first << " id=" << tid
          << "\nlogdet=" << st.logdet
          << "\nΔDopt=" << delta
          << "\nrate=" << rate << "/s";
      if (st.logdet <= thresh_logdet_) oss << "\n[THRESHOLD HIT]";
      txt.text = oss.str();
      txt.lifetime = rclcpp::Duration::from_seconds(text_lifetime_);

      arr.markers.push_back(txt);
    }

    pub_markers_->publish(arr);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DoptTrackingMetrics>());
  rclcpp::shutdown();
  return 0;
}
