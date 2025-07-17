#include <rclcpp/rclcpp.hpp>
#include "crazyflie_yolo/msg/tracked_obstacle_array.hpp"
#include "crazyflie_yolo/msg/tracked_obstacle.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>
#include <map>
#include <vector>
#include <string>
#include <chrono>

using crazyflie_yolo::msg::TrackedObstacle;
using crazyflie_yolo::msg::TrackedObstacleArray;
using namespace std::chrono_literals;

class MultiDroneFusionNode : public rclcpp::Node
{
public:
    MultiDroneFusionNode()
        : Node("multi_drone_fusion_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "✅ Multi-Drone Fusion Node Initialized");

        // Subscribers for each drone's tracked obstacles
        drone_names_ = {"drone1", "drone2", "drone3"};
        for (const auto &drone : drone_names_)
        {
            std::string topic = "/" + drone + "/tracked_obstacles_array";
            RCLCPP_INFO(this->get_logger(), "Subscribing to %s", topic.c_str());
            auto sub = this->create_subscription<TrackedObstacleArray>(
                topic, 10,
                [this](const TrackedObstacleArray::SharedPtr msg)
                {
                    this->fuseDroneObstacles(msg);
                });
            subs_.push_back(sub);
        }

        // Publisher for fused tracked obstacles
        fused_pub_ = this->create_publisher<TrackedObstacleArray>(
            "/fused_tracked_obstacles_array", 10);

        // Timer to periodically publish fused obstacles
        publish_timer_ = this->create_wall_timer(
            500ms, std::bind(&MultiDroneFusionNode::publishFusedObstacles, this));
    }

private:
    std::vector<std::string> drone_names_;
    std::vector<rclcpp::Subscription<TrackedObstacleArray>::SharedPtr> subs_;
    rclcpp::Publisher<TrackedObstacleArray>::SharedPtr fused_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::map<int, TrackedObstacle> fused_obstacles_; // Persistent fused obstacle map

    void fuseDroneObstacles(const TrackedObstacleArray::SharedPtr msg)
    {
        for (const auto &obstacle : msg->obstacles)
        {
            auto it = fused_obstacles_.find(obstacle.id);
            if (it != fused_obstacles_.end())
            {
                // Covariance Intersection fusion
                auto &existing = it->second;
                Eigen::Vector3d z_new(obstacle.position.x, obstacle.position.y, obstacle.position.z);
                Eigen::Vector3d z_existing(existing.position.x, existing.position.y, existing.position.z);

                Eigen::Matrix3d cov_new, cov_existing;
                for (int i = 0; i < 9; ++i)
                {
                    cov_new(i / 3, i % 3) = obstacle.covariance[i];
                    cov_existing(i / 3, i % 3) = existing.covariance[i];
                }

                // Compute omega dynamically based on determinant of covariances
                double det_new = cov_new.determinant();
                double det_existing = cov_existing.determinant();
                double omega = det_new / (det_new + det_existing + 1e-6);

                Eigen::Matrix3d inv_cov_new = cov_new.inverse();
                Eigen::Matrix3d inv_cov_existing = cov_existing.inverse();

                Eigen::Matrix3d cov_fused = (omega * inv_cov_new + (1.0 - omega) * inv_cov_existing).inverse();
                Eigen::Vector3d pos_fused = cov_fused * (omega * inv_cov_new * z_new + (1.0 - omega) * inv_cov_existing * z_existing);

                existing.position.x = pos_fused.x();
                existing.position.y = pos_fused.y();
                existing.position.z = pos_fused.z();
                for (int i = 0; i < 9; ++i)
                {
                    existing.covariance[i] = cov_fused(i / 3, i % 3);
                }
            }
            else
            {
                // Add new obstacle and ensure frame_id is "world"
                TrackedObstacle new_obstacle = obstacle;
                new_obstacle.header.frame_id = "world";
                fused_obstacles_[obstacle.id] = new_obstacle;
            }
        }
    }

    void publishFusedObstacles()
    {
        TrackedObstacleArray fused_msg;
        fused_msg.header.stamp = this->now();
        fused_msg.header.frame_id = "world";

        for (const auto &[id, obstacle] : fused_obstacles_)
        {
            fused_msg.obstacles.push_back(obstacle);
        }

        fused_pub_->publish(fused_msg);
        RCLCPP_INFO(this->get_logger(), "📤 Published %zu fused obstacles", fused_msg.obstacles.size());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiDroneFusionNode>());
    rclcpp::shutdown();
    return 0;
}
