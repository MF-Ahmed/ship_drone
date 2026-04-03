#include <rclcpp/rclcpp.hpp>

#include "crazyflie_yolo/msg/detection3_d_stamped.hpp"
#include "crazyflie_yolo/msg/tracked_obstacle.hpp"
#include "crazyflie_yolo/msg/tracked_obstacle_array.hpp"

#include <Eigen/Dense>
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <limits>
#include <cmath>

using crazyflie_yolo::msg::Detection3DStamped;
using crazyflie_yolo::msg::TrackedObstacle;
using crazyflie_yolo::msg::TrackedObstacleArray;

struct Vec3 { double x=0,y=0,z=0; };
static inline double norm3(const Vec3&a,const Vec3&b){
  const double dx=a.x-b.x, dy=a.y-b.y, dz=a.z-b.z;
  return std::sqrt(dx*dx+dy*dy+dz*dz);
}

// ----- Hungarian (min-cost) for square matrices -----
static std::vector<int> hungarianMinCost(const std::vector<std::vector<double>>& cost)
{
  const int n = (int)cost.size();
  const double INF = 1e18;
  std::vector<double> u(n+1), v(n+1);
  std::vector<int> p(n+1), way(n+1);

  for(int i=1;i<=n;i++){
    p[0]=i;
    int j0=0;
    std::vector<double> minv(n+1, INF);
    std::vector<char> used(n+1, false);
    do{
      used[j0]=true;
      int i0=p[j0], j1=0;
      double delta=INF;
      for(int j=1;j<=n;j++){
        if(used[j]) continue;
        double cur = cost[i0-1][j-1] - u[i0] - v[j];
        if(cur < minv[j]) { minv[j]=cur; way[j]=j0; }
        if(minv[j] < delta) { delta=minv[j]; j1=j; }
      }
      for(int j=0;j<=n;j++){
        if(used[j]) { u[p[j]] += delta; v[j] -= delta; }
        else { minv[j] -= delta; }
      }
      j0=j1;
    }while(p[j0]!=0);

    do{
      int j1=way[j0];
      p[j0]=p[j1];
      j0=j1;
    }while(j0);
  }

  std::vector<int> ans(n, -1);
  for(int j=1;j<=n;j++){
    if(p[j] >= 1 && p[j] <= n) ans[p[j]-1] = j-1;
  }
  return ans;
}

static std::vector<std::vector<double>> makeSquareCost(
  const std::vector<std::vector<double>>& C, double pad_cost)
{
  const int R = (int)C.size();
  const int K = (R>0)? (int)C[0].size() : 0;
  const int n = std::max(R,K);
  std::vector<std::vector<double>> S(n, std::vector<double>(n, pad_cost));
  for(int i=0;i<R;i++)
    for(int j=0;j<K;j++)
      S[i][j]=C[i][j];
  return S;
}

struct Track {
  int id = -1;
  int class_id = -1;
  std::string class_name;
  std::string drone_ns;

  Eigen::Matrix<double,6,1> x;   // [px,py,pz,vx,vy,vz]
  Eigen::Matrix<double,6,6> P;
  rclcpp::Time last_update;
  int missed = 0;
};

class AB3DMOT3DTrackerNode : public rclcpp::Node
{
public:
  AB3DMOT3DTrackerNode() : Node("ab3dmot_3d_tracker_node")
  {
    detection_topic_ = declare_parameter<std::string>("detection_topic", "detections_3d");
    output_topic_    = declare_parameter<std::string>("output_topic", "ab3dmot/tracked_obstacles_array");

    assoc_thresh_    = declare_parameter<double>("association_threshold", 10.0);
    max_missed_      = declare_parameter<int>("max_missed", 10);

    q_process_       = declare_parameter<double>("q_process", 1.0);
    r_meas_          = declare_parameter<double>("r_meas", 0.75);

    require_same_class_ = declare_parameter<bool>("require_same_class", true);

    sub_ = create_subscription<Detection3DStamped>(
      detection_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AB3DMOT3DTrackerNode::cb, this, std::placeholders::_1));

    pub_ = create_publisher<TrackedObstacleArray>(output_topic_, rclcpp::QoS(10));

    RCLCPP_INFO(get_logger(), "AB3DMOT baseline | sub=%s pub=%s",
                detection_topic_.c_str(), output_topic_.c_str());
  }

private:
  void cb(const Detection3DStamped::ConstSharedPtr msg)
  {
    const rclcpp::Time t = msg->header.stamp;
    const double dt = last_stamp_.nanoseconds() > 0
      ? std::max(1e-3, (t - last_stamp_).seconds())
      : 0.2;
    last_stamp_ = t;

    // Build measurements (3D points) from msg (one detection per message in your pipeline)
    // Your Detection3DStamped is “one detection”; we’ll treat each callback as one measurement batch of size 1.
    // If you later publish arrays, adapt easily.
    std::vector<Vec3> meas;
    std::vector<int>  cls;
    std::vector<std::string> cls_name;
    std::vector<std::string> drone_ns;

    meas.push_back(Vec3{msg->position.x, msg->position.y, msg->position.z});
    cls.push_back((int)msg->class_id);
    cls_name.push_back(msg->class_name);
    drone_ns.push_back(msg->drone_ns);

    // 1) Predict all tracks
    predictAll(dt);

    // 2) Associate using Hungarian (AB3DMOT style)
    associateAndUpdate(meas, cls, cls_name, drone_ns, t);

    // 3) Prune old tracks
    prune();

    // 4) Publish
    publish(t);
  }

  void predictAll(double dt)
  {
    Eigen::Matrix<double,6,6> F = Eigen::Matrix<double,6,6>::Identity();
    F(0,3)=dt; F(1,4)=dt; F(2,5)=dt;

    Eigen::Matrix<double,6,6> Q = Eigen::Matrix<double,6,6>::Zero();
    // simple CV noise (position + velocity)
    const double q = q_process_;
    Q(0,0)=q*dt*dt; Q(1,1)=q*dt*dt; Q(2,2)=q*dt*dt;
    Q(3,3)=q*dt;    Q(4,4)=q*dt;    Q(5,5)=q*dt;

    for (auto &tr : tracks_) {
      tr.x = F * tr.x;
      tr.P = F * tr.P * F.transpose() + Q;
      tr.missed++;
    }
  }

  void associateAndUpdate(const std::vector<Vec3>& meas,
                          const std::vector<int>& cls,
                          const std::vector<std::string>& cls_name,
                          const std::vector<std::string>& drone_ns,
                          const rclcpp::Time& t)
  {
    const int M = (int)meas.size();
    const int Tn = (int)tracks_.size();

    if (M == 0) return;
    if (Tn == 0) {
      for (int i=0;i<M;i++) createTrack(meas[i], cls[i], cls_name[i], drone_ns[i], t);
      return;
    }

    // cost matrix Tn x M
    std::vector<std::vector<double>> C(Tn, std::vector<double>(M, 0.0));
    for (int i=0;i<Tn;i++){
      Vec3 p{tracks_[i].x(0), tracks_[i].x(1), tracks_[i].x(2)};
      for (int j=0;j<M;j++){
        double d = norm3(p, meas[j]);
        if (require_same_class_ && tracks_[i].class_id != cls[j]) {
          d += 1e4; // disallow
        }
        C[i][j] = d;
      }
    }

    // Hungarian needs square
    auto Sq = makeSquareCost(C, 1e6);
    auto assign = hungarianMinCost(Sq); // row track -> col meas

    std::vector<bool> meas_used(M, false);

    for (int i=0;i<Tn;i++){
      int j = assign[i];
      if (j < 0 || j >= M) continue;
      if (C[i][j] > assoc_thresh_) continue;
      if (C[i][j] > 1e3) continue; // class-disallowed
      updateTrack(tracks_[i], meas[j], t);
      tracks_[i].class_id = cls[j];
      tracks_[i].class_name = cls_name[j];
      tracks_[i].drone_ns = drone_ns[j];
      meas_used[j] = true;
    }

    // Unmatched measurements -> new tracks
    for (int j=0;j<M;j++){
      if (!meas_used[j]) createTrack(meas[j], cls[j], cls_name[j], drone_ns[j], t);
    }
  }

  void updateTrack(Track& tr, const Vec3& z, const rclcpp::Time& t)
  {
    // H = [I3 0]
    Eigen::Matrix<double,3,6> H = Eigen::Matrix<double,3,6>::Zero();
    H(0,0)=1; H(1,1)=1; H(2,2)=1;

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * (r_meas_ * r_meas_);

    Eigen::Vector3d zv(z.x, z.y, z.z);
    Eigen::Vector3d y = zv - H * tr.x;
    Eigen::Matrix3d S = H * tr.P * H.transpose() + R;
    Eigen::Matrix<double,6,3> K = tr.P * H.transpose() * S.inverse();

    tr.x = tr.x + K * y;
    tr.P = (Eigen::Matrix<double,6,6>::Identity() - K * H) * tr.P;

    tr.last_update = t;
    tr.missed = 0;
  }

  void createTrack(const Vec3& z, int class_id, const std::string& class_name,
                   const std::string& drone_ns, const rclcpp::Time& t)
  {
    Track tr;
    tr.id = next_id_++;
    tr.class_id = class_id;
    tr.class_name = class_name;
    tr.drone_ns = drone_ns;

    tr.x.setZero();
    tr.x(0)=z.x; tr.x(1)=z.y; tr.x(2)=z.z;

    tr.P = Eigen::Matrix<double,6,6>::Identity();
    tr.P.topLeftCorner<3,3>() *= 4.0;
    tr.P.bottomRightCorner<3,3>() *= 10.0;

    tr.last_update = t;
    tr.missed = 0;
    tracks_.push_back(tr);
  }

  void prune()
  {
    tracks_.erase(
      std::remove_if(tracks_.begin(), tracks_.end(),
        [this](const Track& tr){ return tr.missed > max_missed_; }),
      tracks_.end());
  }

  void publish(const rclcpp::Time& stamp)
  {
    TrackedObstacleArray out;
    out.header.stamp = stamp;
    out.header.frame_id = "world";

    for (const auto& tr : tracks_) {
      TrackedObstacle o;
      o.id = tr.id;
      o.class_id = tr.class_id;
      o.class_name = tr.class_name;
      o.drone_ns = tr.drone_ns;

      o.position.x = tr.x(0);
      o.position.y = tr.x(1);
      o.position.z = tr.x(2);

      // store 3x3 position covariance
      for (int i=0;i<9;i++) o.covariance[i] = 0.0;
      o.covariance[0] = tr.P(0,0);
      o.covariance[4] = tr.P(1,1);
      o.covariance[8] = tr.P(2,2);

      out.obstacles.push_back(o);
    }

    pub_->publish(out);
  }

  std::string detection_topic_;
  std::string output_topic_;
  double assoc_thresh_;
  int max_missed_;
  double q_process_;
  double r_meas_;
  bool require_same_class_;

  rclcpp::Subscription<Detection3DStamped>::SharedPtr sub_;
  rclcpp::Publisher<TrackedObstacleArray>::SharedPtr pub_;

  rclcpp::Time last_stamp_;
  std::vector<Track> tracks_;
  int next_id_ = 1;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AB3DMOT3DTrackerNode>());
  rclcpp::shutdown();
  return 0;
}