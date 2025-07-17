// File: src/crazyflie_ekf_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "crazyflie_ekf/crazyflie_ekf.hpp"  // Header for the EKF logic class

class CrazyflieEkfNode : public rclcpp::Node
{
public:
  CrazyflieEkfNode()
  : Node("crazyflie_ekf_node"),
    ekf_()
  {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
       "/imu", 10,
      std::bind(&CrazyflieEkfNode::imuCallback, this, std::placeholders::_1));

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
       "/gps", 10,
      std::bind(&CrazyflieEkfNode::gpsCallback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
       "/odom", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), this->get_namespace() + " EKF Node initialized.");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  CrazyflieEKF ekf_;

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    ekf_.updateImu(msg);
  }

  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    auto now = this->get_clock()->now();
    nav_msgs::msg::Odometry odom_msg = ekf_.computeOdometry(msg, now);

    odom_pub_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom_msg.header;
    tf_msg.child_frame_id = odom_msg.child_frame_id;
    tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
    tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
    tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrazyflieEkfNode>());
  rclcpp::shutdown();
  return 0;
}
