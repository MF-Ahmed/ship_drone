// src/path_tracer.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

class PathTracer : public rclcpp::Node {
public:
  PathTracer() : Node("path_tracer")
  {
    // Parameters
    odom_topic_ = declare_parameter<std::string>("odom_topic", "ekf/odom");
    std::string path_topic = declare_parameter<std::string>("path_topic", "trajectory");
    max_points_ = declare_parameter<int>("max_points", 5000);
    if (max_points_ < 1) max_points_ = 5000;

    log_dir_ = declare_parameter<std::string>(
      "log_dir",
      "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_ekf/path_logs");

    // Resolve namespace (for CSV)
    std::string ns = this->get_namespace();      // e.g. "/drone1"
    if (!ns.empty() && ns.front() == '/') {
      ns.erase(0, 1);
    }
    if (ns.empty()) ns = "root";
    ns_ = ns;

    openCsv();

    // Publisher
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, rclcpp::QoS(10));

    // Initialize path header
    path_.header.frame_id = "world";
    path_.poses.reserve(static_cast<size_t>(max_points_));

    // Subscriber
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(50),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        // Build PoseStamped for Path
        geometry_msgs::msg::PoseStamped p;
        p.header = msg->header;
        p.pose   = msg->pose.pose;

        // Keep frame_id consistent
        if (path_.header.frame_id.empty()) {
          path_.header.frame_id = msg->header.frame_id;
        }

        path_.poses.push_back(p);
        if (static_cast<int>(path_.poses.size()) > max_points_) {
          // drop oldest
          path_.poses.erase(path_.poses.begin());
        }

        // stamp path header (node clock)
        path_.header.stamp = this->get_clock()->now();
        path_pub_->publish(path_);

        // Also log to CSV
        logOdomToCsv(*msg);
      });
  }

  ~PathTracer() override {
    if (csv_.is_open()) {
      csv_.close();
    }
  }

private:
  // ---------- CSV helpers ----------
  void openCsv()
  {
    using std::chrono::system_clock;

    std::filesystem::path base{log_dir_};
    std::error_code ec;
    std::filesystem::create_directories(base, ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(), "Failed to create log_dir '%s': %s",
                   base.string().c_str(), ec.message().c_str());
      return;
    }

    // Build filename: path_<ns>_<YYYYMMDD_HHMMSS>.csv
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);

    std::ostringstream fname;
    fname << "path_" << ns_ << "_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
    std::filesystem::path full = base / fname.str();
    log_path_ = full.string();

    csv_.open(full, std::ios::out);
    if (!csv_) {
      RCLCPP_ERROR(get_logger(), "Could not open CSV file: %s", log_path_.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "PathTracer logging to CSV: %s", log_path_.c_str());

    // Header
    csv_ << "stamp,ns,odom_topic,frame_id,"
         << "pos_x,pos_y,pos_z,"
         << "orient_x,orient_y,orient_z,orient_w,"
         << "lin_vel_x,lin_vel_y,lin_vel_z,"
         << "ang_vel_x,ang_vel_y,ang_vel_z\n";
    csv_.flush();
  }

  void logOdomToCsv(const nav_msgs::msg::Odometry &msg)
  {
    if (!csv_) return;

    const auto &p = msg.pose.pose.position;
    const auto &q = msg.pose.pose.orientation;
    const auto &lv = msg.twist.twist.linear;
    const auto &av = msg.twist.twist.angular;

    // Convert stamp to double seconds
    double t = rclcpp::Time(msg.header.stamp).seconds();

    csv_ << std::fixed << std::setprecision(6)
         << t << ","
         << ns_ << ","
         << odom_topic_ << ","
         << msg.header.frame_id << ","
         << p.x << "," << p.y << "," << p.z << ","
         << q.x << "," << q.y << "," << q.z << "," << q.w << ","
         << lv.x << "," << lv.y << "," << lv.z << ","
         << av.x << "," << av.y << "," << av.z
         << "\n";

    // Optionally flush every line (simple but slightly slower)
    csv_.flush();
  }

  // ---------- Members ----------
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Path path_;
  int max_points_{5000};

  std::string odom_topic_;
  std::string log_dir_;
  std::string log_path_;
  std::string ns_;      // cleaned namespace like "drone1"

  std::ofstream csv_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathTracer>());
  rclcpp::shutdown();
  return 0;
}
