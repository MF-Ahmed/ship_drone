#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <cmath>
#include <string>
#include <chrono>

static double yawFromQuat(const geometry_msgs::msg::Quaternion &q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

class QStarGoalNode : public rclcpp::Node
{
public:
  QStarGoalNode() : rclcpp::Node("qstar_goal_node")
  {
    ns_ = declare_parameter<std::string>("ns", "drone1");

    odom_topic_  = declare_parameter<std::string>("odom_topic",  "/" + ns_ + "/ekf/odom");
    qstar_topic_ = declare_parameter<std::string>("qstar_topic", "/" + ns_ + "/assigned_pose");

    // What to output for your dual_action_client
    speed_     = declare_parameter<double>("speed", 0.20);
    hover_sec_ = declare_parameter<double>("hover_sec", 2.0);
    yaw_goal_  = declare_parameter<double>("yaw_goal", 0.0);

    // Safety / filtering
    max_jump_world_ = declare_parameter<double>("max_jump_world", 1.00); // ignore q* updates that jump too far
    alpha_          = declare_parameter<double>("alpha", 0.25);          // low-pass filter for q*
    min_err_xy_     = declare_parameter<double>("min_err_xy", 0.02);     // don't spam tiny moves

    // Z handling
    use_assigned_z_ = declare_parameter<bool>("use_assigned_z", true);   // use q*.z
    fixed_z_        = declare_parameter<double>("fixed_z", 10.0);        // used if use_assigned_z=false

    // Publish a single topic you can echo & use as "client args"
    publish_goal_vec_ = declare_parameter<bool>("publish_goal_vec", true);
    goal_vec_topic_   = declare_parameter<std::string>("goal_vec_topic", "/" + ns_ + "/qstar_goal_vec");

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(50),
      [this](nav_msgs::msg::Odometry::SharedPtr msg)
      {
        odom_ = *msg;
        have_odom_ = true;
      });

    qstar_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      qstar_topic_, rclcpp::QoS(50),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
        EigenQ q{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};

        if (!have_qstar_raw_) {
          qstar_filt_ = q;
          qstar_last_raw_ = q;
          have_qstar_raw_ = true;
          have_qstar_filt_ = true;
          jumped_ = false;
          return;
        }

        const double jump = dist(q, qstar_last_raw_);
        qstar_last_raw_ = q;

        if (jump > max_jump_world_) {
          jumped_ = true; // ignore this update
          return;
        }

        jumped_ = false;
        qstar_filt_ = lerp(qstar_filt_, q, alpha_);
        have_qstar_filt_ = true;
      });

    if (publish_goal_vec_) {
      goal_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(goal_vec_topic_, rclcpp::QoS(10));
    }

    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this](){ tick(); });

    //RCLCPP_INFO(get_logger(), "qstar_goal_node (DIRECT) up for [%s]", ns_.c_str());
    //RCLCPP_INFO(get_logger(), "  odom:  %s", odom_topic_.c_str());
    //RCLCPP_INFO(get_logger(), "  q*:    %s", qstar_topic_.c_str());
    //RCLCPP_INFO(get_logger(), "  goal:  %s", goal_vec_topic_.c_str());
  }

private:
  struct EigenQ { double x{0}, y{0}, z{0}; };

  static double dist(const EigenQ& a, const EigenQ& b) {
    const double dx=a.x-b.x, dy=a.y-b.y, dz=a.z-b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }

  static EigenQ lerp(const EigenQ& a, const EigenQ& b, double alpha) {
    return EigenQ{
      alpha*b.x + (1.0-alpha)*a.x,
      alpha*b.y + (1.0-alpha)*a.y,
      alpha*b.z + (1.0-alpha)*a.z
    };
  }

  void tick()
  {
    if (!have_odom_ || !have_qstar_filt_) return;

    if (jumped_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[%s] q* jump ignored (>%.2fm). Holding last filtered q*.",
        ns_.c_str(), max_jump_world_);
      return;
    }

    const auto &p = odom_.pose.pose.position;
    const auto &q = odom_.pose.pose.orientation;

    const double px = p.x, py = p.y;
    const double yaw = yawFromQuat(q);

    // filtered world q*
    const EigenQ qstar = qstar_filt_;

    // choose target Z (either q*.z or fixed)
    const double target_z = use_assigned_z_ ? qstar.z : fixed_z_;

    // FULL world error to reach q*
    const double ex = qstar.x - px;
    const double ey = qstar.y - py;
    const double err_xy = std::sqrt(ex*ex + ey*ey);

    if (err_xy < min_err_xy_) return;

    // Convert WORLD error to body distances expected by your server:
    // world = R(yaw) * [dx,dy]  => [dx,dy] = R(-yaw) * world_error
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double dx =  c * ex + s * ey;
    const double dy = -s * ex + c * ey;

    // Publish a vector that directly corresponds to dual_action_client args:
    // vector.x = distance_x, vector.y = distance_y, vector.z = target_z (ABSOLUTE)
    if (publish_goal_vec_ && goal_pub_) {
      geometry_msgs::msg::Vector3Stamped v;
      v.header.stamp = now();
      v.header.frame_id = ns_ + "/base_footprint";
      v.vector.x = dx;
      v.vector.y = dy;
      v.vector.z = target_z;
      goal_pub_->publish(v);
    }

    // Print ready-to-use command (throttled)
    //RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      //"[%s] q*=(%.3f,%.3f,%.3f) cur=(%.3f,%.3f) err_xy=%.2f | distance_x=%.3f distance_y=%.3f | yaw=%.2fdeg",
      //ns_.c_str(), qstar.x, qstar.y, qstar.z, px, py, err_xy, dx, dy, yaw * 180.0 / M_PI);

    //RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      //"ros2 run crazyflie_servers dual_action_client --ros-args "
      //"-p ns:=%s -p target_z:=%.3f -p hover_sec:=%.2f -p yaw:=%.3f "
      //"-p distance_x:=%.6f -p distance_y:=%.6f -p speed:=%.2f",
      //ns_.c_str(), target_z, hover_sec_, yaw_goal_, dx, dy, speed_);
  }

  // params
  std::string ns_;
  std::string odom_topic_;
  std::string qstar_topic_;

  double speed_{0.2};
  double hover_sec_{2.0};
  double yaw_goal_{0.0};

  double max_jump_world_{1.0};
  double alpha_{0.25};
  double min_err_xy_{0.02};

  bool use_assigned_z_{true};
  double fixed_z_{10.0};

  bool publish_goal_vec_{true};
  std::string goal_vec_topic_;

  // ros
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr qstar_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry odom_;
  bool have_odom_{false};

  bool have_qstar_raw_{false};
  bool have_qstar_filt_{false};
  bool jumped_{false};

  EigenQ qstar_last_raw_{};
  EigenQ qstar_filt_{};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QStarGoalNode>());
  rclcpp::shutdown();
  return 0;
}
