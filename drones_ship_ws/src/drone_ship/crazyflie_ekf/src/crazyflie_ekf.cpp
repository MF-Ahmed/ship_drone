#include "crazyflie_ekf/crazyflie_ekf.hpp"
#include "crazyflie_ekf/gps2enu.hpp"

namespace crazyflie_ekf
{

CrazyflieEKF::CrazyflieEKF(rclcpp::Node::SharedPtr node)
: node_(node)
{
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    node_->get_namespace() + std::string("/imu"), 10,
    std::bind(&CrazyflieEKF::imuCallback, this, std::placeholders::_1));

  gps_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
    node_->get_namespace() + std::string("/gps"), 10,
    std::bind(&CrazyflieEKF::gpsCallback, this, std::placeholders::_1));

  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
     node_->get_namespace() + std::string("/ekf/odom"), 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  std::string node_name = node_->get_name();  // e.g., "crazyflie_ekf_node"

  node_->declare_parameter("ref_lat", 0.0);
  node_->declare_parameter("ref_lon", 0.0);
  node_->declare_parameter("ref_alt", 0.0);
  node_->declare_parameter("process_noise_v", 0.01);        // Velocity noise
  node_->declare_parameter("process_noise_bax", 1e-4);      // Accelerometer bias x noise
  node_->declare_parameter("process_noise_bay", 1e-4);      // Accelerometer bias y noise

  node_->declare_parameter("gps_noise", 5.0);
  node_->declare_parameter("base_frame", node_->get_namespace() + std::string("/base_footprint"));
  node_->declare_parameter("gps_noise_xy", 5.0);
  node_->declare_parameter("gps_noise_z", 10.0);  // more noise vertically


  double ref_lat = node_->get_parameter("ref_lat").as_double();
  double ref_lon = node_->get_parameter("ref_lon").as_double();
  double ref_alt = node_->get_parameter("ref_alt").as_double();
  double gps_noise_xy = node_->get_parameter("gps_noise_xy").as_double();
  double gps_noise_z  = node_->get_parameter("gps_noise_z").as_double();
  double process_noise_v   = node_->get_parameter("process_noise_v").as_double();
  double process_noise_bax = node_->get_parameter("process_noise_bax").as_double();
  double process_noise_bay = node_->get_parameter("process_noise_bay").as_double();
  RCLCPP_INFO(node_->get_logger(), "[%s] ref_lat: %.6f", node_name.c_str(), ref_lat);
  RCLCPP_INFO(node_->get_logger(), "[%s] ref_lon: %.6f", node_name.c_str(), ref_lon);
  RCLCPP_INFO(node_->get_logger(), "[%s] ref_alt: %.2f", node_name.c_str(), ref_alt);
  RCLCPP_INFO(node_->get_logger(), "[%s] Using GPS ENU Origin: lat=%.6f, lon=%.6f, alt=%.2f",
            node_name.c_str(), ref_lat, ref_lon, ref_alt);


  gps_converter_.initialize(ref_lat, ref_lon, ref_alt);

  x_ = Eigen::VectorXd::Zero(8);  // [x, y, z, vx, vy, vz, bax, bay]
  P_ = Eigen::MatrixXd::Identity(8, 8);
  F_ = Eigen::MatrixXd::Identity(8, 8);
  Q_ = Eigen::MatrixXd::Zero(8, 8);
  Q_.block<3,3>(3,3) = process_noise_v * Eigen::Matrix3d::Identity();   // vx, vy, vz
  Q_(6,6) = process_noise_bax;
  Q_(7,7) = process_noise_bay;

  // Measurement matrix H: maps state [x, y, z, vx, vy, vz, bax, bay] to GPS [x, y, z]
  H_ = Eigen::MatrixXd::Zero(3, 8);
  H_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
  // GPS noise covariance
  R_ = Eigen::MatrixXd::Zero(3, 3);
  R_(0, 0) = gps_noise_xy;  // noise in x (east)
  R_(1, 1) = gps_noise_xy;  // noise in y (north)
  R_(2, 2) = gps_noise_z;   // larger noise in z (up)


  RCLCPP_INFO(node_->get_logger(), "%s EKF node initialized", node_->get_namespace());
  RCLCPP_INFO(node_->get_logger(), "Reference Altitude: %.2f", ref_alt);

}

void CrazyflieEKF::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  //RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "IMU Accel: [%.2f, %.2f, %.2f]  Gyro: [%.2f, %.2f, %.2f]",
    msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z,
    msg->angular_velocity.x,
    msg->angular_velocity.y,
    msg->angular_velocity.z;
    
  last_imu_msg_ = msg;
}

void CrazyflieEKF::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (!last_imu_msg_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Waiting for IMU data...");
    return;
  }

  if (!gps_converter_.isInitialized()) {
    RCLCPP_WARN(node_->get_logger(), "GPS converter not initialized.");
    gps_converter_.initialize(msg->latitude, msg->longitude, msg->altitude);
    //RCLCPP_INFO(node_->get_logger(), "Set dynamic UTM origin: lat=%.6f, lon=%.6f, alt=%.2f", msg->latitude, msg->longitude, msg->altitude);
    return;
  }

  //RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
    //"GPS Fix: lat=%.6f, lon=%.6f, alt=%.2f",
    //msg->latitude,
    //msg->longitude,
    //msg->altitude);

  auto now = node_->now();
  double dt = has_last_position_ ? (now - last_time_).seconds() : 0.05;
  last_time_ = now;
  has_last_position_ = true;

  // Predict step with acceleration
  // Update F matrix to reflect bias effect on velocity
  F_.block<3,3>(0,3) = dt * Eigen::Matrix3d::Identity();  // dx += v * dt
  F_(3,6) = -dt;  // vx affected by bax
  F_(4,7) = -dt;  // vy affected by bay

  // Apply IMU data for prediction
  if (last_imu_msg_) {
    double ax = last_imu_msg_->linear_acceleration.x;
    double ay = last_imu_msg_->linear_acceleration.y;
    double az = last_imu_msg_->linear_acceleration.z - 9.81;

    // Apply bias correction
    ax -= x_(6);  // subtract estimated bax
    ay -= x_(7);  // subtract estimated bay

    // Limit extreme values
    ax = std::clamp(ax, -10.0, 10.0);
    ay = std::clamp(ay, -10.0, 10.0);
    az = std::clamp(az, -10.0, 10.0);

    // Predict velocity
    x_(3) += dt * ax;
    x_(4) += dt * ay;
    x_(5) += dt * az;

    // Predict position
    x_(0) += dt * x_(3);
    x_(1) += dt * x_(4);
    x_(2) += dt * x_(5);
  } else {
    x_ = F_ * x_;
  }

  // Covariance prediction
  P_ = F_ * P_ * F_.transpose() + Q_;


  // Convert GPS to ENU
  geometry_msgs::msg::Point enu = gps_converter_.convert(msg->latitude, msg->longitude, msg->altitude);

  double jump_threshold = 30.0;  // meters
  if (has_last_position_) {
    double dx = enu.x - last_position_.x;
    double dy = enu.y - last_position_.y;
    double dz = enu.z - last_position_.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (dist > jump_threshold) {
      RCLCPP_WARN(node_->get_logger(), "GPS jump detected (%.1f m) — ignoring!", dist);
      return;
    }
  }
  last_position_ = enu;

  Eigen::Vector3d z;
  z << enu.x, enu.y, enu.z;

  //RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Converted ENU: [%.2f, %.2f, %.2f]", enu.x, enu.y, enu.z);

  // Update step
  Eigen::Vector3d y = z - H_ * x_;
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;


  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(8, 8);  // match state size
  P_ = (I - K * H_) * P_;

  std::string base_frame = node_->get_parameter("base_frame").as_string();

  //RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "EKF Estimate: [%.2f, %.2f, %.2f]  Vel: [%.2f, %.2f, %.2f]", x_(0), x_(1), x_(2), x_(3), x_(4), x_(5));


  // Publish odometry
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = now;
  odom_msg.header.frame_id =  node_->get_namespace() + std::string("/odom");
  odom_msg.child_frame_id = base_frame;
  odom_msg.pose.pose.position.x = x_(0);
  odom_msg.pose.pose.position.y = x_(1);
  odom_msg.pose.pose.position.z = x_(2);
  odom_msg.pose.pose.orientation = last_imu_msg_->orientation;
  odom_msg.twist.twist.linear.x = x_(3);
  odom_msg.twist.twist.linear.y = x_(4);
  odom_msg.twist.twist.linear.z = x_(5);
  odom_pub_->publish(odom_msg);

  // Broadcast TF
  if (dt > 0.05) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom_msg.header;
    tf_msg.child_frame_id = base_frame;
    tf_msg.transform.translation.x = x_(0);
    tf_msg.transform.translation.y = x_(1);
    tf_msg.transform.translation.z = x_(2);
    tf_msg.transform.rotation = last_imu_msg_->orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }
}

}  // namespace crazyflie_ekf
