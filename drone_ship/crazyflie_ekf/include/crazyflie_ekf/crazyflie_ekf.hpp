#ifndef CRAZYFLIE_EKF__CRAZYFLIE_EKF_HPP_
#define CRAZYFLIE_EKF__CRAZYFLIE_EKF_HPP_

#include <memory>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "crazyflie_ekf/gps2enu.hpp"

namespace crazyflie_ekf
{

class CrazyflieEKF
{
public:
  explicit CrazyflieEKF(rclcpp::Node::SharedPtr node);

private:
  // Callback functions
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  // Node handle
  rclcpp::Node::SharedPtr node_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

  // Publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // IMU data
  geometry_msgs::msg::Vector3 last_angular_velocity_;
  geometry_msgs::msg::Vector3 last_linear_acceleration_;
  sensor_msgs::msg::Imu::SharedPtr last_imu_msg_;

  // GPS and pose tracking
  geometry_msgs::msg::Point last_position_;
  rclcpp::Time last_time_;
  bool has_last_position_ = false;

  // Coordinate converter
  Gps2Enu gps_converter_;

  // EKF state
  Eigen::VectorXd x_;  // size 8: [x y z vx vy vz bax bay]
  Eigen::MatrixXd P_;  // 8x8 covariance matrix
  Eigen::MatrixXd F_;  // 8x8 state transition matrix
  Eigen::MatrixXd Q_;  // 8x8 process noise matrix
  Eigen::MatrixXd H_;  // 3x8 measurement matrix (GPS position only)
  Eigen::MatrixXd R_;  // 3x3 measurement noise covariance (for GPS)

};

}  // namespace crazyflie_ekf

#endif  // CRAZYFLIE_EKF__CRAZYFLIE_EKF_HPP_
