// multi_drone_fusion_node.cpp
#include <rclcpp/rclcpp.hpp>
#include "crazyflie_yolo/msg/tracked_obstacle_array.hpp"
#include "crazyflie_yolo/msg/tracked_obstacle.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>
#include <map>
#include <vector>
#include <string>
#include <chrono>
#include <limits>
#include <unordered_map>
#include <algorithm>
#include <tuple>
#include <set>
#include <array>
#include <fstream>      // NEW: CSV logging
#include <iomanip>      // NEW: formatting for CSV
#include <filesystem>  // add at the top

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

        // ===== Parameters =====
        drone_names_ = this->declare_parameter<std::vector<std::string>>(
            "drone_names", std::vector<std::string>{"drone1","drone2","drone3"});

        topic_format_ = this->declare_parameter<std::string>(
            "topic_format", "/%s/tracked_obstacles_array");

        association_threshold_ = this->declare_parameter<double>(
            "association_threshold", 40.0);

        prune_time_s_ = this->declare_parameter<double>(
            "prune_time_s", 5.0);

        aquabot_frame_ = this->declare_parameter<std::string>(
            "aquabot_frame", "aquabot/base_link");

        // NEW: CSV logging params
        log_path_   = this->declare_parameter<std::string>("log_path", "/tmp/fused_obstacles.csv");
        log_append_ = this->declare_parameter<bool>("log_append", false);
        openCsv_();

        // ===== Subscribers for each drone =====
        for (const auto &drone : drone_names_) {
            char buf[256];
            std::snprintf(buf, sizeof(buf), topic_format_.c_str(), drone.c_str());
            std::string topic = buf;
            RCLCPP_INFO(this->get_logger(), "Subscribing to %s", topic.c_str());

            auto sub = this->create_subscription<TrackedObstacleArray>(
                topic, 10,
                [this](const TrackedObstacleArray::SharedPtr msg)
                {
                    this->ingestDroneArray(msg);
                });
            subs_.push_back(sub);
        }

        // ===== Publishers =====
        fused_pub_ = this->create_publisher<TrackedObstacleArray>("/fused_tracked_obstacles_array", 10);
        fused_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/fused_tracked_obstacles", 10);
        fused_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/fused_tracked_obstacle_markers", 10);

        // ===== Timers =====
        fuse_timer_ = this->create_wall_timer(100ms, std::bind(&MultiDroneFusionNode::runFusion, this));
        publish_timer_ = this->create_wall_timer(200ms, std::bind(&MultiDroneFusionNode::publishFusedObstacles, this));

        RCLCPP_INFO(this->get_logger(),
                    "Params: association_threshold=%.1f, prune_time=%.1fs, aquabot_frame=%s, log_path=%s, append=%s",
                    association_threshold_, prune_time_s_, aquabot_frame_.c_str(),
                    log_path_.c_str(), log_append_ ? "true" : "false");
    }

    ~MultiDroneFusionNode() override {
        if (ofs_.is_open()) ofs_.close();
    }

private:
    // ==== Color utilities for class & drone tinting ====
    struct RGB { double r, g, b; };

    static constexpr std::array<RGB, 5> kClassPalette = {{
        {1.00, 0.20, 0.20},  // class 0 → red
        {0.20, 1.00, 0.30},  // class 1 → green
        {0.20, 0.55, 1.00},  // class 2 → blue
        {1.00, 0.65, 0.20},  // class 3 → orange
        {0.70, 0.20, 1.00},  // class 4 → violet
    }};

    static inline RGB blend(const RGB& a, const RGB& b, double t) {
        return { a.r*(1.0 - t) + b.r*t, a.g*(1.0 - t) + b.g*t, a.b*(1.0 - t) + b.b*t };
    }
    static inline RGB colorForClass(int cls) {
        if (cls >= 0 && cls < static_cast<int>(kClassPalette.size())) return kClassPalette[cls];
        return {1.0, 1.0, 1.0};
    }
    static inline int droneIndexFromNS(std::string ns) {
        if (!ns.empty() && ns.front() == '/') ns.erase(0, 1);
        if (ns.rfind("drone1", 0) == 0) return 1;
        if (ns.rfind("drone2", 0) == 0) return 2;
        if (ns.rfind("drone3", 0) == 0) return 3;
        return 0;
    }
    static inline RGB tintByDrone(const RGB& base, const std::string& ns) {
        const int idx = droneIndexFromNS(ns);
        if (idx == 2) return blend(base, {1.0, 1.0, 1.0}, 0.25); // lighten
        if (idx == 3) return blend(base, {0.0, 0.0, 0.0}, 0.35); // darken
        return base;
    }
    static inline RGB colorByClassAndDrone(int cls, const std::string& ns) {
        return tintByDrone(colorForClass(cls), ns);
    }

    // ===== Types & storage =====
    struct FusedTrack
    {
        int id;                               // unique fused id
        int class_id;                         // label 0..4
        std::set<std::string> drones_seen;    // provenance (contributors)
        Eigen::Vector3d pos;                  // fused position
        Eigen::Matrix3d P;                    // fused 3x3 covariance
        rclcpp::Time last_update;
    };

    // Staged input (cleared every runFusion)
    std::vector<TrackedObstacle> inbox_;

    // Fused tracks
    std::map<int, FusedTrack> fused_tracks_;
    int next_fused_id_ = 0;

    // ROS I/O
    std::vector<std::string> drone_names_;
    std::vector<rclcpp::Subscription<TrackedObstacleArray>::SharedPtr> subs_;
    rclcpp::Publisher<TrackedObstacleArray>::SharedPtr fused_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr fused_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fused_marker_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr fuse_timer_;

    // Params
    std::string topic_format_;
    double association_threshold_;
    double prune_time_s_;
    std::string aquabot_frame_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // ===== CSV logging =====
    std::string log_path_;
    bool log_append_ = false;
    std::ofstream ofs_;

    void openCsv_() {
        std::filesystem::path p(log_path_);
        std::filesystem::create_directories(p.parent_path());

        std::ios::openmode mode = std::ios::out | (log_append_ ? std::ios::app : std::ios::trunc);
        ofs_.open(log_path_, mode);
        if (!ofs_) {
            RCLCPP_ERROR(get_logger(), "Failed to open CSV log file: %s", log_path_.c_str());
            return;
        }
        if (!log_append_) {
            ofs_ << "stamp_ns,node_ns,src_drone_ns,fused_id,class_id,frame,"
                    "x,y,z,"
                    "cov00,cov01,cov02,"
                    "cov10,cov11,cov12,"
                    "cov20,cov21,cov22\n";
            ofs_.flush();
        }
    }

    // ===== Helpers =====
    static inline Eigen::Matrix3d covFromMsg9(const std::array<double,9> &c)
    {
        Eigen::Matrix3d C;
        for (int i=0;i<3;i++) for (int j=0;j<3;j++) C(i,j) = c[i*3+j];
        return C;
    }

    static inline double mahalanobis2(const Eigen::Vector3d &dz, const Eigen::Matrix3d &S)
    {
        Eigen::Matrix3d Sinv = S.inverse();
        return (dz.transpose() * Sinv * dz)(0,0);
    }

    // ===== Input ingestion =====
    void ingestDroneArray(const TrackedObstacleArray::SharedPtr msg)
    {
        for (const auto &o : msg->obstacles) {
            inbox_.push_back(o);
        }
    }

    // ===== Fusion cycle =====
    void runFusion()
    {
        const rclcpp::Time now = this->now();

        // 1) Prune old fused tracks
        {
            std::vector<int> erase_ids;
            for (const auto &kv : fused_tracks_) {
                double dt = (now - kv.second.last_update).seconds();
                if (dt > prune_time_s_) erase_ids.push_back(kv.first);
            }
            for (int id : erase_ids) {
                fused_tracks_.erase(id);
                RCLCPP_DEBUG(get_logger(), "Pruned fused track %d", id);
            }
        }

        if (inbox_.empty()) return;

        // 2) Associate & fuse (per class)
        for (const auto &obs : inbox_) {
            const int cls = obs.class_id;                   // assumes field in msg
            const std::string drone_ns = obs.drone_ns;      // assumes field in msg

            Eigen::Vector3d z(obs.position.x, obs.position.y, obs.position.z);
            Eigen::Matrix3d Rm = covFromMsg9(obs.covariance);

            int best_id = -1;
            double best_d2 = std::numeric_limits<double>::infinity();

            for (const auto &kv : fused_tracks_) {
                const auto &ft = kv.second;
                if (ft.class_id != cls) continue;           // gate by class

                Eigen::Vector3d dz = z - ft.pos;
                Eigen::Matrix3d S = Rm;                     // simple gate
                double d2 = mahalanobis2(dz, S);

                if (d2 < association_threshold_ && d2 < best_d2) {
                    best_d2 = d2;
                    best_id = ft.id;
                }
            }

            if (best_id >= 0) {
                // Update matched track via Covariance Intersection
                auto &ft = fused_tracks_.at(best_id);

                Eigen::Matrix3d P_new = Rm;
                Eigen::Matrix3d P_old = ft.P;

                double det_new = P_new.determinant();
                double det_old = P_old.determinant();
                double omega = det_new / (det_new + det_old + 1e-12);
                omega = std::clamp(omega, 0.0, 1.0);

                Eigen::Matrix3d inv_new = P_new.inverse();
                Eigen::Matrix3d inv_old = P_old.inverse();

                Eigen::Matrix3d P_fused = (omega * inv_new + (1.0 - omega) * inv_old).inverse();
                Eigen::Vector3d pos_fused = P_fused * (omega * inv_new * z + (1.0 - omega) * inv_old * ft.pos);

                ft.pos = pos_fused;
                ft.P   = P_fused;
                ft.last_update = now;
                ft.drones_seen.insert(drone_ns);
            } else {
                // New fused track
                FusedTrack ft;
                ft.id = next_fused_id_++;
                ft.class_id = cls;
                ft.pos = z;
                ft.P = Rm;
                ft.last_update = now;
                ft.drones_seen.clear();
                ft.drones_seen.insert(drone_ns);
                fused_tracks_[ft.id] = ft;
            }
        }

        inbox_.clear();
    }

    // ===== Publisher (arrays + markers + CSV logging) =====
    void publishFusedObstacles()
    {
        TrackedObstacleArray fused_msg;
        fused_msg.header.stamp = this->now();
        fused_msg.header.frame_id = "world";

        geometry_msgs::msg::PoseArray poses;
        poses.header = fused_msg.header;

        visualization_msgs::msg::MarkerArray markers;
        int marker_id = 0;

        // Try get Aquabot pose for distances (optional)
        bool have_aquabot = false;
        Eigen::Vector3d aquabot_world(0,0,0);
        try {
            auto tf = tf_buffer_.lookupTransform("world", aquabot_frame_, tf2::TimePointZero);
            aquabot_world = Eigen::Vector3d(tf.transform.translation.x,
                                            tf.transform.translation.y,
                                            tf.transform.translation.z);
            have_aquabot = true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "No TF world->%s: %s (fused marker ranges show n/a)",
                                 aquabot_frame_.c_str(), ex.what());
        }

        // compute timestamp once for CSV rows
        const int64_t stamp_ns = rclcpp::Time(fused_msg.header.stamp).nanoseconds();



        const std::string node_ns = this->get_namespace();

        for (const auto &[id, ft] : fused_tracks_)
        {
            // ---- Populate TrackedObstacleArray ----
            TrackedObstacle o;
            o.header = fused_msg.header;
            o.id = id;
            o.position.x = ft.pos.x();
            o.position.y = ft.pos.y();
            o.position.z = ft.pos.z();
            for (int i=0;i<3;i++)
                for (int j=0;j<3;j++)
                    o.covariance[i*3+j] = ft.P(i,j);

            o.class_id = ft.class_id;
            if (!ft.drones_seen.empty())
                o.drone_ns = *ft.drones_seen.begin(); // pick one contributor
            else
                o.drone_ns = "";

            fused_msg.obstacles.push_back(o);

            // ---- Debug PoseArray ----
            geometry_msgs::msg::Pose p;
            p.position.x = o.position.x;
            p.position.y = o.position.y;
            p.position.z = o.position.z;
            p.orientation.w = 1.0;
            poses.poses.push_back(p);

            // ---- Markers (sphere + text) ----
            std::string tint_ns = o.drone_ns;

            visualization_msgs::msg::Marker sphere;
            sphere.header = fused_msg.header;
            sphere.ns = "fused_tracked_obstacles";
            sphere.id = marker_id++;
            sphere.type = visualization_msgs::msg::Marker::SPHERE;
            sphere.action = visualization_msgs::msg::Marker::ADD;
            sphere.pose = p;
            sphere.scale.x = 0.6; sphere.scale.y = 0.6; sphere.scale.z = 0.6;
            sphere.color.a = 0.95;
            {
                RGB col = colorByClassAndDrone(o.class_id, tint_ns);
                sphere.color.r = col.r;
                sphere.color.g = col.g;
                sphere.color.b = col.b;
            }
            sphere.lifetime = rclcpp::Duration::from_seconds(0.5);
            markers.markers.push_back(sphere);

            visualization_msgs::msg::Marker label;
            label.header = fused_msg.header;
            label.ns = "fused_tracked_obstacles_text";
            label.id = marker_id++;
            label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            label.action = visualization_msgs::msg::Marker::ADD;
            label.pose = p;
            label.pose.position.z += 0.8;
            label.scale.z = 0.4;
            label.color.a = 1.0;
            label.color.r = 1.0; label.color.g = 1.0; label.color.b = 1.0;
            label.lifetime = rclcpp::Duration::from_seconds(0.5);

            double range_m = std::numeric_limits<double>::quiet_NaN();
            double range_sigma_m = std::numeric_limits<double>::quiet_NaN();
            if (have_aquabot) {
                Eigen::Vector3d delta(ft.pos.x() - aquabot_world.x(),
                                      ft.pos.y() - aquabot_world.y(),
                                      ft.pos.z() - aquabot_world.z());
                range_m = delta.norm();
                if (range_m > 1e-6) {
                    Eigen::Vector3d u = delta / range_m;
                    double var_r = (u.transpose() * ft.P * u)(0,0);
                    range_sigma_m = (var_r > 0.0) ? std::sqrt(var_r) : 0.0;
                } else {
                    double var_max = std::max({ft.P(0,0), ft.P(1,1), ft.P(2,2)});
                    range_sigma_m = (var_max > 0.0) ? std::sqrt(var_max) : 0.0;
                }
            }

            char buff[200];
            if (have_aquabot && std::isfinite(range_m)) {
                std::snprintf(buff, sizeof(buff),
                              "ftrk:%d cls:%d dr:%s\nR:%.1fm \u00B1%.1fm",
                              o.id, o.class_id, tint_ns.c_str(),
                              range_m, range_sigma_m);
            } else {
                std::snprintf(buff, sizeof(buff),
                              "ftrk:%d cls:%d dr:%s\nR:n/a",
                              o.id, o.class_id, tint_ns.c_str());
            }
            label.text = buff;
            markers.markers.push_back(label);

            // ========= PRINT to console (namespace, covariance, class id) =========
            const auto &P = ft.P;
            //RCLCPP_INFO(get_logger(),
                //"NS(node)=%s | SRC(drone)=%s | fused_id=%d | class=%d | pos=(%.2f, %.2f, %.2f) | "
                //"Cov=[ [%.3e %.3e %.3e] [%.3e %.3e %.3e] [%.3e %.3e %.3e] ]",
                //node_ns.c_str(), o.drone_ns.c_str(), o.id, o.class_id,
                //o.position.x, o.position.y, o.position.z,
                //P(0,0), P(0,1), P(0,2),
                //P(1,0), P(1,1), P(1,2),
                //P(2,0), P(2,1), P(2,2));

            // ===================== CSV LOG ROW =====================
            if (ofs_) {
                // Use fixed for positions, scientific for covariance for readability
                ofs_ << stamp_ns << ','
                     << node_ns << ','
                     << o.drone_ns << ','
                     << o.id << ','
                     << o.class_id << ','
                     << fused_msg.header.frame_id << ','
                     << std::fixed << std::setprecision(6)
                     << o.position.x << ','
                     << o.position.y << ','
                     << o.position.z << ','
                     << std::scientific << std::setprecision(6)
                     << P(0,0) << ',' << P(0,1) << ',' << P(0,2) << ','
                     << P(1,0) << ',' << P(1,1) << ',' << P(1,2) << ','
                     << P(2,0) << ',' << P(2,1) << ',' << P(2,2) << '\n';
            }
        }

        if (ofs_) ofs_.flush();

        fused_pub_->publish(fused_msg);
        fused_pose_pub_->publish(poses);
        fused_marker_pub_->publish(markers);

        //RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             //"📤 Published %zu fused obstacles (+ markers, + csv)",
                             //fused_msg.obstacles.size());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiDroneFusionNode>());
    rclcpp::shutdown();
    return 0;
}
