#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <GeographicLib/UTMUPS.hpp>
#include <Eigen/Dense>
#include <vector>

using std::placeholders::_1;

class EKFNode : public rclcpp::Node {
public:
  EKFNode()
  : Node("ekf_node"), got_gps_(false), got_imu_(false) {
    // Declare and assign parameters to member variables
    double q_pos = declare_parameter("q_pos", 0.01);
    double q_vel = declare_parameter("q_vel", 0.01);
    double r_gps = declare_parameter("r_gps", 5.0);
    double r_gps_x = declare_parameter("r_gps_x", r_gps);
    double r_gps_y = declare_parameter("r_gps_y", r_gps);
    double r_gps_z = declare_parameter("r_gps_z", r_gps);
    gps_x_offset_ = declare_parameter("gps_x_offset", 0.0);
    gps_y_offset_ = declare_parameter("gps_y_offset", 0.0);

    RCLCPP_INFO(this->get_logger(), "gps_x_offset = %f", gps_x_offset_);
    RCLCPP_INFO(this->get_logger(), "gps_y_offset = %f", gps_y_offset_);

    std::vector<double> init_state = declare_parameter("initial_state", std::vector<double>{0, 0, 0, 0, 0, 0});
    utm_origin_easting_ = declare_parameter("utm_origin_easting", 0.0);
    utm_origin_northing_ = declare_parameter("utm_origin_northing", 0.0);
    utm_origin_altitude_ = declare_parameter("utm_origin_altitude", 0.0);

    if (init_state.size() == 6) {
      for (size_t i = 0; i < 6; ++i) x_(i) = init_state[i];
    }
    RCLCPP_INFO(this->get_logger(), "Initial state x_: [%.3f, %.3f, %.3f]", x_(0), x_(1), x_(2));

    Q_ = Eigen::MatrixXd::Zero(6, 6);
    Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * q_pos;
    Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * q_vel;

    R_ = Eigen::Matrix3d::Zero();
    R_(0, 0) = r_gps_x;
    R_(1, 1) = r_gps_y;
    R_(2, 2) = r_gps_z;

    P_ = Eigen::MatrixXd::Identity(6, 6);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/aquabot/imu", 10, std::bind(&EKFNode::imu_callback, this, _1));
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/aquabot/sensors/gps/gps/fix", 10, std::bind(&EKFNode::gps_callback, this, _1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/aquabot/ekf/odometry", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    prev_time_ = now();
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!got_gps_) return;

    rclcpp::Time now = msg->header.stamp;
    double dt = (now - prev_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) {
      prev_time_ = now;
      return;
    }
    prev_time_ = now;

    Eigen::Vector3d acc_filtered_ = Eigen::Vector3d::Zero();
    double acc_alpha_ = 0.1;  // low-pass filter coefficient


    Eigen::Vector3d acc_raw(msg->linear_acceleration.x,
                        msg->linear_acceleration.y,
                        msg->linear_acceleration.z - 9.81); // gravity compensation


    acc_filtered_ = acc_alpha_ * acc_raw + (1.0 - acc_alpha_) * acc_filtered_;  // low pass filter                    

    x_.segment<3>(3) += acc_filtered_ * dt;
    x_.segment<3>(0) += x_.segment<3>(3) * dt;

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 3) = dt; F(1, 4) = dt; F(2, 5) = dt;
    P_ = F * P_ * F.transpose() + Q_;

    publish_odometry(now, msg->orientation);
    got_imu_ = true;
  }

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) return;

    double easting, northing;
    int zone; bool northp;
    try {
      GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude, zone, northp, easting, northing);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "UTM conversion failed: %s", e.what());
      return;
    }


    if (!got_gps_) {
      utm_origin_easting_ = easting;
      utm_origin_northing_ = northing;
      utm_origin_altitude_ = msg->altitude;
      RCLCPP_INFO(get_logger(), "Set dynamic UTM origin: e=%.3f, n=%.3f, alt=%.3f", easting, northing, msg->altitude);
    }

    double local_x = (easting - utm_origin_easting_) - gps_x_offset_;
    double local_y = (northing - utm_origin_northing_) - gps_y_offset_;
    double local_z = msg->altitude - utm_origin_altitude_;

    //RCLCPP_INFO(this->get_logger(), "ENU raw: [%.3f, %.3f], origin: [%.3f, %.3f], offset: [%.3f, %.3f]",  easting, northing, utm_origin_easting_, utm_origin_northing_, gps_x_offset_, gps_y_offset_);
    //RCLCPP_INFO(this->get_logger(),  "GPS ENU: [%.2f, %.2f, %.2f], x_: [%.2f, %.2f, %.2f]",  local_x, local_y, local_z, x_(0), x_(1), x_(2));

    //RCLCPP_INFO(this->get_logger(),  "GPS raw altitude: %.2f | Local Z: %.2f",  msg->altitude, local_z);

    Eigen::Vector3d z(local_x, local_y, local_z);
    Eigen::Vector3d z_pred = x_.segment<3>(0);
    Eigen::Vector3d y = z - z_pred;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ += K * y;
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
    got_gps_ = true;
  }

  void publish_odometry(const rclcpp::Time &stamp, const geometry_msgs::msg::Quaternion &orientation) {
    if (!got_gps_) return;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "aquabot/odom";
    odom.child_frame_id = "aquabot/base_link";

    odom.pose.pose.position.x = x_(0);
    odom.pose.pose.position.y = x_(1);
    odom.pose.pose.position.z = x_(2);
    odom.pose.pose.orientation = orientation;

    odom.twist.twist.linear.x = x_(3);
    odom.twist.twist.linear.y = x_(4);
    odom.twist.twist.linear.z = x_(5);

    //RCLCPP_INFO(this->get_logger(), "EKF pose: x=%.3f, y=%.3f, z=%.3f | vx=%.3f, vy=%.3f, vz=%.3f", x_(0), x_(1), x_(2), x_(3), x_(4), x_(5));

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "aquabot/odom";
    tf.child_frame_id = "aquabot/base_link";
    tf.transform.translation.x = x_(0);
    tf.transform.translation.y = x_(1);
    tf.transform.translation.z = x_(2);
    tf.transform.rotation = orientation;

    tf_broadcaster_->sendTransform(tf);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  Eigen::VectorXd x_ = Eigen::VectorXd::Zero(6);
  Eigen::MatrixXd P_ = Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd Q_;
  Eigen::Matrix3d R_;

  rclcpp::Time prev_time_;
  double utm_origin_easting_;
  double utm_origin_northing_;
  double utm_origin_altitude_;
  double gps_x_offset_;
  double gps_y_offset_;
  bool got_gps_, got_imu_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFNode>());
  rclcpp::shutdown();
  return 0;
}
