#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "crazyflie_yolo/msg/tracked_obstacle.hpp"
#include "crazyflie_yolo/msg/tracked_obstacle_array.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Dense>
#include <vector>
#include <map>
#include <cmath>




using crazyflie_yolo::msg::TrackedObstacle;
using crazyflie_yolo::msg::TrackedObstacleArray;

using namespace std::chrono_literals;

class MultiObstacleTracker : public rclcpp::Node
{
public:
    MultiObstacleTracker()
        : Node("multi_obstacle_tracker"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          next_track_id_(0)
    {


        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        
        // Log it for confirmation
        if (use_sim_time) {
            RCLCPP_INFO(this->get_logger(), "✅ use_sim_time is ENABLED.");
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ use_sim_time is DISABLED.");
        }
        // Subscribe to obstacle detections
        detection_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "obstacle_points", 50,
            std::bind(&MultiObstacleTracker::detectionCallback, this, std::placeholders::_1));

        tracked_pub_ = this->create_publisher<TrackedObstacleArray>(
            "tracked_obstacles_array", 10);


        // Publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("tracked_obstacles", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tracked_obstacle_markers", 10);




        // Timer for periodic pruning
        timer_ = this->create_wall_timer(200ms, std::bind(&MultiObstacleTracker::pruneOldTracks, this));

        RCLCPP_INFO(this->get_logger(), "Multi-Obstacle Tracker Node Initialized");

        // Set container ground truth (replace with actual values if needed)
        container_gt_ = Eigen::Vector3d(12.805, -2.646, -1.795); // Example from gz model -m floating_container -p
    }

private:
    struct ObstacleTrack
    {
        int id;
        Eigen::Vector3d position;    // [x, y, z]
        Eigen::Matrix3d covariance;         // 3x3 covariance
        rclcpp::Time last_update;
    };

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr detection_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<TrackedObstacleArray>::SharedPtr tracked_pub_; 



    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::map<int, ObstacleTrack> tracks_;
    int next_track_id_;

    Eigen::Vector3d container_gt_;  // Ground truth container position

    // Parameters
    double association_threshold_ = 30.0; // Mahalanobis distance threshold
    double measurement_noise_ = 0.5;
    double track_prune_time_ = 3.0; // seconds

    void detectionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        Eigen::Vector3d detection(msg->point.x, msg->point.y, 0.0); // Clamp z=0

        RCLCPP_INFO(this->get_logger(), 
            "Received detection: x=%.2f y=%.2f z=%.2f", 
            msg->point.x, msg->point.y, msg->point.z);

        // Data association
        int best_track_id = -1;
        double best_mahalanobis = std::numeric_limits<double>::max();

        for (auto &[id, track] : tracks_)
        {
            Eigen::Vector3d diff = detection - track.position;
            Eigen::Matrix3d S = track.covariance + Eigen::Matrix3d::Identity() * measurement_noise_;

            double mahalanobis = diff.transpose() * S.inverse() * diff;

            if (mahalanobis < association_threshold_ && mahalanobis < best_mahalanobis)
            {
                best_mahalanobis = mahalanobis;
                best_track_id = id;
            }
        }

        if (best_track_id >= 0)
        {
            // Update existing track
            updateTrack(tracks_[best_track_id], detection, msg->header.stamp);
        }
        else
        {
            // Create new track
            createTrack(detection, msg->header.stamp);
        }
    }

    void updateTrack(ObstacleTrack &track, const Eigen::Vector3d &z, const rclcpp::Time &stamp)
    {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * measurement_noise_;
        Eigen::Vector3d y = z - track.position;
        Eigen::Matrix3d S = track.covariance + R;
        Eigen::Matrix3d K = track.covariance * S.inverse();

        track.position += K * y;
        track.covariance = (Eigen::Matrix3d::Identity() - K) * track.covariance; 
        track.last_update = stamp;

        RCLCPP_INFO(this->get_logger(), 
            "Updated Track ID %d to x=%.2f y=%.2f z=%.2f",
            track.id, track.position(0), track.position(1), track.position(2));
    }

    void createTrack(const Eigen::Vector3d &z, const rclcpp::Time &stamp)
    {
        ObstacleTrack track;
        track.id = next_track_id_++;
        track.position = z;
        track.covariance = Eigen::Matrix3d::Identity() * 2.0; // Initial uncertainty
        track.last_update = stamp;

        tracks_[track.id] = track;
        RCLCPP_INFO(this->get_logger(), "Created new Track ID %d", track.id);
    }

    void pruneOldTracks()
    {
        rclcpp::Time now = this->now();
        std::vector<int> to_remove;

        for (const auto &[id, track] : tracks_)
        {
            double dt = (now - track.last_update).seconds();
            if (dt > track_prune_time_)
            {
                RCLCPP_INFO(this->get_logger(), "Pruned Track ID %d", id);
                to_remove.push_back(id);
            }
        }

        for (int id : to_remove)
            tracks_.erase(id);

        publishTracks();
    }

    void publishTracks()
    {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = this->now();
        pose_array.header.frame_id = "world";

        TrackedObstacleArray tracked_array;
        tracked_array.header.stamp = this->now();
        tracked_array.header.frame_id = "world";




        visualization_msgs::msg::MarkerArray marker_array;
        int marker_id = 0;

        // Lookup Aquabot position
        geometry_msgs::msg::TransformStamped aquabot_tf;
        try
        {
            aquabot_tf = tf_buffer_.lookupTransform("world", "aquabot/base_link", tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get Aquabot pose: %s", ex.what());
            return;
        }

        for (const auto &[id, track] : tracks_)
        {
            TrackedObstacle tracked;
            tracked.id = id;
            tracked.position.x = track.position(0);
            tracked.position.y = track.position(1);
            tracked.position.z = track.position(2);
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    tracked.covariance[i * 3 + j] = track.covariance(i, j);
                }
            }            
                            
            tracked_array.obstacles.push_back(tracked); 
        
            // Compute XY distance to Aquabot
            double dx = track.position(0) - aquabot_tf.transform.translation.x;
            double dy = track.position(1) - aquabot_tf.transform.translation.y;
            double distance_xy = std::sqrt(dx * dx + dy * dy);

            // Compute error to container ground truth
            Eigen::Vector3d diff = track.position - container_gt_;
            double error = diff.norm();

            RCLCPP_INFO(this->get_logger(),
                "[TRACK] ID: %d | (x=%.2f, y=%.2f) | Dist to Aquabot: %.2f m | Error to Container: %.2f m",
                id, track.position(0), track.position(1), distance_xy, error);

            // Add pose
            geometry_msgs::msg::Pose pose;
            pose.position.x = track.position(0);
            pose.position.y = track.position(1);
            pose.position.z = 0.0; // Clamp z=0
            pose.orientation.w = 1.0;
            pose_array.poses.push_back(pose);

            // Sphere marker
            visualization_msgs::msg::Marker sphere;
            sphere.header = pose_array.header;
            sphere.ns = "tracked_obstacles";
            sphere.id = marker_id++;
            sphere.type = visualization_msgs::msg::Marker::SPHERE;
            sphere.action = visualization_msgs::msg::Marker::ADD;
            sphere.pose = pose;
            sphere.scale.x = 0.5;
            sphere.scale.y = 0.5;
            sphere.scale.z = 0.5;
            sphere.lifetime = rclcpp::Duration::from_seconds(0.0);

            // Color coding: red if close to ground truth, green otherwise
            if (error < 1.0) {
                sphere.color.r = 1.0;
                sphere.color.g = 0.0;
                sphere.color.b = 0.0; // RED if error < 1m
            } else {
                sphere.color.r = 0.0;
                sphere.color.g = 1.0;
                sphere.color.b = 0.0; // GREEN otherwise
            }
            sphere.color.a = 1.0;
            marker_array.markers.push_back(sphere);

            // Text marker for ID and error
            visualization_msgs::msg::Marker text_marker = sphere;
            text_marker.id = marker_id++;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.text = "Track " + std::to_string(track.id) +
                               "\nError: " + std::to_string(error).substr(0,5) + "m";
            text_marker.pose.position.z += 1.0; // offset text
            text_marker.scale.z = 0.4;
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.lifetime = rclcpp::Duration::from_seconds(0.0);
            marker_array.markers.push_back(text_marker);
        }

        pose_pub_->publish(pose_array);
        marker_pub_->publish(marker_array);
        tracked_pub_->publish(tracked_array);
        //RCLCPP_INFO(this->get_logger(), "Published %zu markers", marker_array.markers.size());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiObstacleTracker>());
    rclcpp::shutdown();
    return 0;
}
