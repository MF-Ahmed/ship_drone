#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <iomanip>
#include <sstream>
#include <limits>

class DroneOdomLogger : public rclcpp::Node
{
public:
  DroneOdomLogger()
  : Node("drone_odom_logger")
  {
    // Folder to store CSV files (can be overridden via parameter / launch)
    log_dir_ = declare_parameter<std::string>(
      "log_dir",
      "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs");

    openCsv();

    auto qos = rclcpp::SensorDataQoS();

    // drone1
    subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
      "/drone1/ekf/odom", qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg){
        odomCallback(msg, "drone1", "ekf");
      }));

    subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
      "/drone1/odom/raw", qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg){
        odomCallback(msg, "drone1", "raw");
      }));

    // drone2
    subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
      "/drone2/ekf/odom", qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg){
        odomCallback(msg, "drone2", "ekf");
      }));

    subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
      "/drone2/odom/raw", qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg){
        odomCallback(msg, "drone2", "raw");
      }));

    // drone3
    subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
      "/drone3/ekf/odom", qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg){
        odomCallback(msg, "drone3", "ekf");
      }));

    subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
      "/drone3/odom/raw", qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg){
        odomCallback(msg, "drone3", "raw");
      }));

    RCLCPP_INFO(get_logger(), "DroneOdomLogger started. Logging to: %s", log_path_.c_str());
  }

  ~DroneOdomLogger() override
  {
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }
  }

private:
  std::string log_dir_;
  std::string log_path_;
  std::ofstream csv_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subs_;

  void openCsv()
  {
    std::filesystem::path base{log_dir_};
    std::error_code ec;
    std::filesystem::create_directories(base, ec);
    if (ec) {
      RCLCPP_ERROR(get_logger(),
        "Failed to create log dir '%s': %s",
        base.string().c_str(), ec.message().c_str());
      return;
    }

    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream fname;
    fname << "drone_odom_"
          << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";

    log_path_ = (base / fname.str()).string();

    csv_.open(log_path_, std::ios::out);
    if (!csv_) {
      RCLCPP_ERROR(get_logger(), "Could not open CSV file: %s", log_path_.c_str());
      return;
    }

    // Header: one row per odom message
    csv_ << "stamp,drone,source,"
         << "x,y,z,"
         << "qx,qy,qz,qw,yaw,"
         << "vx,vy,vz\n";
    csv_.flush();
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg,
                    const std::string &drone,
                    const std::string &source)
  {
    if (!csv_) return;

    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    const auto &twist = msg->twist.twist;

    // Compute yaw from quaternion
    tf2::Quaternion q_tf(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);

    double stamp_sec = msg->header.stamp.sec +
                       msg->header.stamp.nanosec * 1e-9;

    csv_ << std::fixed << std::setprecision(6)
         << stamp_sec << ","
         << drone << ","
         << source << ","
         << p.x << "," << p.y << "," << p.z << ","
         << q.x << "," << q.y << "," << q.z << "," << q.w << ","
         << yaw << ","
         << twist.linear.x << "," << twist.linear.y << "," << twist.linear.z
         << "\n";

    // Optional low-frequency console log
    RCLCPP_DEBUG(get_logger(),
      "[%s %s] t=%.3f pos=(%.2f,%.2f,%.2f) yaw=%.2f",
      drone.c_str(), source.c_str(), stamp_sec, p.x, p.y, p.z, yaw);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneOdomLogger>());
  rclcpp::shutdown();
  return 0;
}
