#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <filesystem>
#include <array>
#include <limits>
#include <algorithm>
#include <string>
#include <vector>

#include "crazyflie_yolo/msg/detection3_d_stamped.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using crazyflie_yolo::msg::Detection3DStamped;

class StereoObstacleLocalizer : public rclcpp::Node
{
public:
    StereoObstacleLocalizer()
    : Node("stereo_obstacle_localizer"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        using namespace message_filters;

        // --- Parameters ---
        save_dir_ = declare_parameter<std::string>(
            "save_dir",
            "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/src/images");
        save_interval_sec_ = declare_parameter<double>("save_interval_sec", 2.5);
        enable_save_frames_ = declare_parameter<bool>("enable_save_frames", true);
        expected_alt_m_ = declare_parameter<double>("expected_alt_m", 10.0);
        alt_tolerance_m_ = declare_parameter<double>("alt_tolerance_m", 3.0);

        std::filesystem::create_directories(save_dir_ + "/rgb");
        std::filesystem::create_directories(save_dir_ + "/disparity");
        std::filesystem::create_directories(save_dir_ + "/depth_mm");

        last_saved_time_ = this->now();
        save_interval_ = rclcpp::Duration::from_seconds(save_interval_sec_);

        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        if (use_sim_time) {
            RCLCPP_INFO(this->get_logger(), "✅ use_sim_time is ENABLED.");
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ use_sim_time is DISABLED.");
        }

        namespace_ = this->get_namespace();

        // --- Subscribers (message_filters for sync) ---
        left_sub_.subscribe(this, "downward_left_camera/image_raw");
        right_sub_.subscribe(this, "downward_right_camera/image_raw");
        yolo_sub_.subscribe(this, "yolo_detections");

        sync_ = std::make_shared<Synchronizer<SyncPolicy>>(SyncPolicy(100), left_sub_, right_sub_, yolo_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(1.0));
        sync_->registerCallback(std::bind(&StereoObstacleLocalizer::callback, this, _1, _2, _3));

        // Camera info
        left_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "downward_left_camera/camera_info", 10,
            std::bind(&StereoObstacleLocalizer::leftInfoCallback, this, _1));
        right_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "downward_right_camera/camera_info", 10,
            std::bind(&StereoObstacleLocalizer::rightInfoCallback, this, _1));

        // Publishers
        detection_pub_ = this->create_publisher<Detection3DStamped>("detections_3d", 10);
        marker_pub_    = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", 10);
        depth_pub_     = this->create_publisher<sensor_msgs::msg::Image>("depth/image", 10);
        depth_vis_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth/image_colored", 10);

        // NEW: disparity publishers (grayscale + optional binary)
        disparity_pub_    = this->create_publisher<sensor_msgs::msg::Image>("disparity/image_mono", 10);
        disparity_bw_pub_ = this->create_publisher<sensor_msgs::msg::Image>("disparity/image_bw", 10);

        RCLCPP_INFO(this->get_logger(), "Stereo Obstacle Localizer Node Initialized");
    }

private:
    // ==== Message filter Sync ====
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image,
        vision_msgs::msg::Detection2DArray> SyncPolicy;

    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_, right_sub_;
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> yolo_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // ==== ROS I/O ====
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_, right_info_sub_;
    rclcpp::Publisher<Detection3DStamped>::SharedPtr detection_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_vis_pub_;

    // NEW
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_pub_;    // mono8
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_bw_pub_; // mono8 (binary)

    // ==== TF ====
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // ==== Saving ====
    std::string save_dir_;
    bool enable_save_frames_{false};
    double save_interval_sec_{2.5};
    rclcpp::Time last_saved_time_;
    rclcpp::Duration save_interval_{rclcpp::Duration::from_seconds(2.5)};
    int frame_counter_ = 0;
    cv::Size rect_size_;
    bool maps_ready_ = false;

    // ==== Depth/Altitude checks ====
    double expected_alt_m_{10.0};
    double alt_tolerance_m_{3.0};

    // ==== Stereo calibration ====
    cv::Mat K1_, D1_, K2_, D2_;
    cv::Mat R_, T_, R1_, R2_, P1_, P2_, Q_;
    cv::Mat map1x_, map1y_, map2x_, map2y_;
    bool info_ready_ = false;

    // ==== Visualization / Markers ====
    visualization_msgs::msg::MarkerArray marker_array_;
    std::string namespace_;

    // ===== Class-color palette (YOLO class IDs 0..4) =====
    struct RGB { double r, g, b; };
    static inline const std::array<RGB,5> kClassPalette {{
        {1.0, 0.0, 0.0},   // 0 → Red
        {0.0, 1.0, 0.0},   // 1 → Green
        {0.0, 0.0, 1.0},   // 2 → Blue
        {1.0, 0.65, 0.0},  // 3 → Orange
        {0.7, 0.0, 1.0},   // 4 → Violet
    }};

    static inline RGB colorForClass(int cls) {
        if (cls >= 0 && cls < static_cast<int>(kClassPalette.size())) return kClassPalette[cls];
        return {1.0, 1.0, 1.0}; // fallback (white)
    }
    static inline RGB blend(const RGB& a, const RGB& b, double t) {
        return { a.r * (1.0 - t) + b.r * t,
                 a.g * (1.0 - t) + b.g * t,
                 a.b * (1.0 - t) + b.b * t };
    }
    static inline int droneIndexFromNS(const std::string& ns_in) {
        std::string s = ns_in;
        if (!s.empty() && s.front() == '/') s.erase(0, 1);
        if (s.rfind("drone1", 0) == 0) return 1;
        if (s.rfind("drone2", 0) == 0) return 2;
        if (s.rfind("drone3", 0) == 0) return 3;
        return 0;
    }
    static inline RGB tintByDrone(const RGB& base, int drone_idx) {
        if (drone_idx == 2) return blend(base, {1.0, 1.0, 1.0}, 0.25);
        if (drone_idx == 3) return blend(base, {0.0, 0.0, 0.0}, 0.35);
        return base;
    }

    // Utility: clean namespace + camera frame
    std::string cameraFrame() const {
        std::string ns_str = std::string(this->get_namespace());
        if (!ns_str.empty() && ns_str.front() == '/') ns_str.erase(0, 1);
        return ns_str + "/downward_left_camera_link";
    }

    // ==== Camera info callbacks ====
    void leftInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        K1_ = cv::Mat(3,3,CV_64F,(void*)msg->k.data()).clone();
        D1_ = cv::Mat(1, (int)msg->d.size(), CV_64F, (void*)msg->d.data()).clone();
        R_  = cv::Mat::eye(3,3,CV_64F);

        RCLCPP_INFO_ONCE(this->get_logger(), "Left camera info received");
        checkCalibrationReady((int)msg->width, (int)msg->height);
    }

    void rightInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        K2_ = cv::Mat(3,3,CV_64F,(void*)msg->k.data()).clone();
        D2_ = cv::Mat(1, (int)msg->d.size(), CV_64F, (void*)msg->d.data()).clone();

        // Baseline: left->right = -B along x (meters)
        T_ = (cv::Mat_<double>(3,1) << -0.025, 0.0, 0.0);

        RCLCPP_INFO_ONCE(this->get_logger(), "Right camera info received");
        checkCalibrationReady((int)msg->width, (int)msg->height);
    }

    void checkCalibrationReady(int width, int height)
    {
        if (info_ready_) return;
        double alpha = 1.0;
        cv::Rect roi1, roi2;
        if (!K1_.empty() && !K2_.empty() && !D1_.empty() && !D2_.empty() && !R_.empty())
        {
            cv::Size img_size((width  > 0 && height > 0) ? width  : 640,
                              (width  > 0 && height > 0) ? height : 480);
            cv::stereoRectify(K1_, D1_, K2_, D2_, img_size, R_, T_, R1_, R2_, P1_, P2_, Q_,
                              cv::CALIB_ZERO_DISPARITY, alpha, img_size, &roi1, &roi2) ;
            cv::initUndistortRectifyMap(K1_, D1_, R1_, P1_, img_size, CV_32FC1, map1x_, map1y_);
            cv::initUndistortRectifyMap(K2_, D2_, R2_, P2_, img_size, CV_32FC1, map2x_, map2y_);
            info_ready_ = true;

            // print once only
            RCLCPP_INFO_ONCE(this->get_logger(), "Stereo rectification ready (size %dx%d)", img_size.width, img_size.height);
            RCLCPP_INFO(this->get_logger(),
            "ROI left=(x=%d, y=%d, w=%d, h=%d), right=(x=%d, y=%d, w=%d, h=%d)",
            roi1.x, roi1.y, roi1.width, roi1.height,
            roi2.x, roi2.y, roi2.width, roi2.height);
        }
    }

    // ==== Altitude helper ====
    double getCameraAltitudeMeters(const std::string& cam_frame)
    {
        try {
            auto tf = tf_buffer_.lookupTransform("world", cam_frame, tf2::TimePointZero,
                                                 tf2::durationFromSec(0.2));
            return tf.transform.translation.z;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF altitude lookup failed: %s", ex.what());
            return std::numeric_limits<double>::quiet_NaN();
        }
    }

    // ==== Main synchronized callback ====
    void callback(
        const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &right_msg,
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr &dets_msg)
    {
        if (!info_ready_) return;

        cv::Mat left = cv_bridge::toCvCopy(left_msg, "bgr8")->image;
        cv::Mat right = cv_bridge::toCvCopy(right_msg, "bgr8")->image;

        // Rectify
        cv::Mat left_rect, right_rect;
        cv::remap(left, left_rect, map1x_, map1y_, cv::INTER_LINEAR);
        cv::remap(right, right_rect, map2x_, map2y_, cv::INTER_LINEAR);

        // Grayscale
        cv::Mat left_gray, right_gray;
        cv::cvtColor(left_rect, left_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_rect, right_gray, cv::COLOR_BGR2GRAY);

        // --- Stereo SGBM ---
        int min_disp   = 0;
        int num_disp   = 192;   // multiple of 16
        int block_size = 7;

        auto sgbm = cv::StereoSGBM::create(min_disp, num_disp, block_size);
        sgbm->setP1(8 * block_size * block_size);
        sgbm->setP2(64 * block_size * block_size);
        sgbm->setPreFilterCap(31);
        sgbm->setUniquenessRatio(7);
        sgbm->setDisp12MaxDiff(1);
        sgbm->setSpeckleWindowSize(100);
        sgbm->setSpeckleRange(32);
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

        cv::Mat disparity_raw, disparity;  // raw: CV_16S (x16), disparity: CV_32F (px)
        sgbm->compute(left_gray, right_gray, disparity_raw);
        disparity_raw.convertTo(disparity, CV_32F, 1.0f / 16.0f);

        // ---- Disparity visualization (grayscale mono8) ----
        float disp_clip_max = 16.0f;    // tune for contrast
        cv::Mat disp_clipped;
        cv::min(disparity, disp_clip_max, disp_clipped);
        cv::Mat disp_norm_u8;
        disp_clipped.convertTo(disp_norm_u8, CV_8U, 255.0 / disp_clip_max);

        auto disp_mono_msg = cv_bridge::CvImage(left_msg->header, "mono8", disp_norm_u8).toImageMsg();
        disp_mono_msg->header.frame_id = cameraFrame();
        disparity_pub_->publish(*disp_mono_msg);

        // ---- Optional: strict black/white (binary) ----
        float disp_thresh = 2.0f; // px
        cv::Mat disp_binary;
        cv::threshold(disparity, disp_binary, disp_thresh, 255.0, cv::THRESH_BINARY);
        disp_binary.convertTo(disp_binary, CV_8U);

        auto disp_bw_msg = cv_bridge::CvImage(left_msg->header, "mono8", disp_binary).toImageMsg();
        disp_bw_msg->header.frame_id = disp_mono_msg->header.frame_id;
        disparity_bw_pub_->publish(*disp_bw_msg);

        // Reproject to 3D
        cv::Mat points3D; // CV_32FC3
        cv::reprojectImageTo3D(disparity, points3D, Q_);

        // === Depth (meters) extraction from points3D Z ===
        cv::Mat depth_m(points3D.size(), CV_32FC1);
        for (int y = 0; y < points3D.rows; ++y) {
            const cv::Vec3f* p3 = points3D.ptr<cv::Vec3f>(y);
            float* pd = depth_m.ptr<float>(y);
            const float* pdsp = disparity.ptr<float>(y);
            for (int x = 0; x < points3D.cols; ++x) {
                if (pdsp[x] > 0.0f && std::isfinite(p3[x][2]) && p3[x][2] > 0.0f && p3[x][2] < 1000.0f) {
                    pd[x] = p3[x][2]; // meters
                } else {
                    pd[x] = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }

        // === Ground-band depth median (vs TF altitude) ===
        {
            double fx = P1_.at<double>(0,0);
            double B  = std::abs(T_.at<double>(0,0));
            std::string cam_frame = cameraFrame();
            double cam_alt = getCameraAltitudeMeters(cam_frame);

            if (fx > 0.0 && B > 0.0 && std::isfinite(cam_alt) && cam_alt > 0.0) {
                double d_pred = (fx * B) / cam_alt;   // expected ground disparity (px)
                double d_lo = std::max(0.0, d_pred * 0.40);
                double d_hi = d_pred * 2.50;

                int y0 = static_cast<int>(disparity.rows * 0.60);
                std::vector<float> vals; 
                vals.reserve(disparity.cols * (disparity.rows - y0) / 4);

                const int stride = 4;
                for (int y = y0; y < disparity.rows; y += stride) {
                    const float* Zp = depth_m.ptr<float>(y);
                    const float* Dp = disparity.ptr<float>(y);
                    for (int x = 0; x < disparity.cols; x += stride) {
                        float Z = Zp[x];
                        float d = Dp[x];
                        if (std::isfinite(Z) && Z > 0.0f && Z < 30.0f &&
                            std::isfinite(d) && d >= d_lo && d <= d_hi) {
                            vals.push_back(Z);
                        }
                    }
                }

                if (!vals.empty()) {
                    std::nth_element(vals.begin(), vals.begin() + vals.size()/2, vals.end());
                    float medZ = vals[vals.size()/2];
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Ground-band depth median ≈ %.2f m (alt=%.2f m, d_pred=%.2f px, n=%zu)",
                        medZ, cam_alt, d_pred, vals.size());
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "No valid pixels in ground-band mask (alt=%.2f m).", cam_alt);
                }
            }
        }

        // Publish depth (32FC1, meters)
        auto depth_msg = cv_bridge::CvImage(left_msg->header, "32FC1", depth_m).toImageMsg();
        depth_msg->header.frame_id = cameraFrame();
        depth_pub_->publish(*depth_msg);

        // Colorized depth for RViz (clip to 0..20 m for visualization)
        cv::Mat depth_clipped = depth_m.clone();
        cv::threshold(depth_clipped, depth_clipped, 20.0, 20.0, cv::THRESH_TRUNC);
        cv::Mat depth_norm_u8;
        cv::normalize(depth_clipped, depth_norm_u8, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::Mat depth_color;
        cv::applyColorMap(depth_norm_u8, depth_color, cv::COLORMAP_TURBO);

        auto depth_vis_msg = cv_bridge::CvImage(left_msg->header, "bgr8", depth_color).toImageMsg();
        depth_vis_msg->header.frame_id = depth_msg->header.frame_id;
        depth_vis_pub_->publish(*depth_vis_msg);

        // Optional: preview windows (comment-out when headless)
         cv::imshow("Disparity mono", disp_norm_u8);
         cv::imshow("Disparity BW", disp_binary);
         cv::imshow("Depth [m] (0..20m colorized)", depth_color);
         cv::waitKey(1);

        // ---- Frame-level altitude sanity check ----
        {
            std::vector<float> vals;
            vals.reserve(depth_m.total());
            for (int y = 0; y < depth_m.rows; ++y) {
                const float* pd = depth_m.ptr<float>(y);
                for (int x = 0; x < depth_m.cols; ++x) {
                    float v = pd[x];
                    if (std::isfinite(v)) vals.push_back(v);
                }
            }
            if (!vals.empty()) {
                std::nth_element(vals.begin(), vals.begin()+vals.size()/2, vals.end());
                float median_depth = vals[vals.size()/2];

                std::string cam_frame = cameraFrame();
                double cam_alt = getCameraAltitudeMeters(cam_frame);

                if (std::isfinite(cam_alt)) {
                    double diff = std::abs(median_depth - cam_alt);
                    if (diff <= alt_tolerance_m_) {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "✅ Depth median %.2f m ≈ camera altitude %.2f m (∆=%.2f m)",
                            median_depth, cam_alt, diff);
                    } else {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "⚠️ Depth median %.2f m differs from camera altitude %.2f m (∆=%.2f m). Check calib/Q/baseline.",
                            median_depth, cam_alt, diff);
                    }
                } else {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Depth median this frame: %.2f m", median_depth);
                }
            }
        }

        // ---- Detections: compute 3D, publish markers + 3D msg, per-ROI depth stats ----
        int id_offset = static_cast<int>(marker_array_.markers.size());
        int id = 0;

        for (const auto &det : dets_msg->detections)
        {
            int cx = static_cast<int>(det.bbox.center.position.x);
            int cy = static_cast<int>(det.bbox.center.position.y);

            if (cx < 0 || cx >= disparity.cols || cy < 0 || cy >= disparity.rows) continue;

            // 7x7 ROI around center (clamped)
            cv::Rect roi(
                std::max(0, cx - 3), std::max(0, cy - 3),
                std::min(7, disparity.cols - std::max(0, cx - 3)),
                std::min(7, disparity.rows - std::max(0, cy - 3))
            );

            cv::Mat disp_roi = disparity(roi);
            int valid_count = cv::countNonZero(disp_roi > 0);
            if (valid_count < static_cast<int>(roi.area() * 0.1)) continue;

            cv::Scalar mean_disp = cv::mean(disp_roi, disp_roi > 0);
            float disp = static_cast<float>(mean_disp[0]);
            if (disp <= 1.0f) continue;

            cv::Vec3f point = points3D.at<cv::Vec3f>(cy, cx);
            if (!std::isfinite(point[2]) || point[2] < 0.1f || point[2] > 20.0f) continue;

            // --- Per-detection robust depth stats (median + IQR) ---
            cv::Mat depth_roi = depth_m(roi);
            std::vector<float> dvals; dvals.reserve(depth_roi.total());
            for (int yy = 0; yy < depth_roi.rows; ++yy) {
                const float* pd = depth_roi.ptr<float>(yy);
                for (int xx = 0; xx < depth_roi.cols; ++xx) {
                    float v = pd[xx];
                    if (std::isfinite(v)) dvals.push_back(v);
                }
            }
            if (!dvals.empty()) {
                auto nth = [&](size_t k){ std::nth_element(dvals.begin(), dvals.begin()+k, dvals.end()); return dvals[k]; };
                std::vector<float> tmp = dvals;
                float med = nth(tmp.size()/2);
                float q1 = nth(dvals.size()/4);
                float q3 = nth(3*dvals.size()/4);
                float iqr = q3 - q1;

                bool ok = (med >= expected_alt_m_ - alt_tolerance_m_) && (med <= expected_alt_m_ + alt_tolerance_m_);
                RCLCPP_INFO(this->get_logger(),
                    "[%s] depth med=%.2f m (IQR=%.2f) — %s  (roi %d×%d @ %d,%d)",
                    (!det.results.empty()? det.results[0].hypothesis.class_id.c_str() : "?"),
                    med, iqr, ok ? "OK≈10m" : "OFF", roi.width, roi.height, roi.x, roi.y);
            }

            // --- Color selection per class + drone tint ---
            std::string ns = std::string(this->get_namespace());
            if (!ns.empty() && ns.front() == '/') ns.erase(0, 1);
            int drone_idx = droneIndexFromNS(ns);

            int cls_id = -1;
            std::string cls_str = "?";
            if (!det.results.empty()) {
                cls_str = det.results[0].hypothesis.class_id;
                try { cls_id = std::stoi(cls_str); } catch (...) { cls_id = -1; }
            }
            RGB base = colorForClass(cls_id);
            RGB col  = tintByDrone(base, drone_idx);

            // Transform to world
            geometry_msgs::msg::PointStamped p_cam;
            p_cam.header = det.header;
            p_cam.header.frame_id = ns + "/downward_left_camera_link";
            p_cam.header.stamp = this->get_clock()->now();
            p_cam.point.x = point[0];
            p_cam.point.y = point[1];
            p_cam.point.z = point[2];

            try {
                geometry_msgs::msg::PointStamped p_world;
                tf_buffer_.transform(p_cam, p_world, "world", tf2::durationFromSec(0.5));

                // Publish 3D detection
                Detection3DStamped det3d;
                det3d.header.stamp = this->now();
                det3d.header.frame_id = "world";
                det3d.position.x = p_world.point.x;
                det3d.position.y = p_world.point.y;
                det3d.position.z = p_world.point.z;

                if (!det.results.empty()) {
                    try { det3d.class_id = std::stoi(det.results[0].hypothesis.class_id); }
                    catch (...) { det3d.class_id = -1; }
                    det3d.score = det.results[0].hypothesis.score;
                } else {
                    det3d.class_id = -1;
                    det3d.score = 0.0f;
                }

                det3d.drone_ns = ns;
                detection_pub_->publish(det3d);

                // Sphere marker at z=0 (sea/deck)
                visualization_msgs::msg::Marker marker;
                marker.header = p_world.header;
                marker.header.frame_id = "world";
                marker.ns = ns + "_obstacles";
                marker.id = id_offset + id * 2;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = p_world.point.x;
                marker.pose.position.y = p_world.point.y;
                marker.pose.position.z = 0.0; // clamp to sea level
                marker.pose.orientation.w = 1.0;
                marker.scale.x = marker.scale.y = marker.scale.z = 0.25;
                marker.color.a = 1.0;
                marker.color.r = col.r;
                marker.color.g = col.g;
                marker.color.b = col.b;
                marker.lifetime = rclcpp::Duration::from_seconds(0.0);
                marker_array_.markers.push_back(marker);

                // Text label
                visualization_msgs::msg::Marker text_marker = marker;
                text_marker.id = id_offset + id * 2 + 1;
                text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                text_marker.text = cls_str;
                text_marker.pose.position.z += 1.0;
                text_marker.scale.z = 0.4;
                text_marker.color.r = 1.0;
                text_marker.color.g = 1.0;
                text_marker.color.b = 1.0;
                marker_array_.markers.push_back(text_marker);

                id++;
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform point to world frame: %s", ex.what());
            }
        }

        marker_pub_->publish(marker_array_);

        // ---- Optional saving of RGB / disparity / depth ----
        if (enable_save_frames_ && (this->now() - last_saved_time_) >= save_interval_) {
            std::string rgb_filename  = save_dir_ + "/rgb/frame_" + std::to_string(frame_counter_) + ".jpg";
            std::string disp_filename = save_dir_ + "/disparity/frame_" + std::to_string(frame_counter_) + ".png";
            std::string depth_filename= save_dir_ + "/depth_mm/frame_" + std::to_string(frame_counter_) + ".png";

            // Save RGB
            cv::imwrite(rgb_filename, left_rect);

            // Save the same grayscale disparity we publish
            cv::imwrite(disp_filename, disp_norm_u8);

            // Save depth in millimeters as 16UC1 (0 for invalid)
            cv::Mat depth_mm_u16(depth_m.size(), CV_16UC1, cv::Scalar(0));
            for (int y = 0; y < depth_m.rows; ++y) {
                const float* pd = depth_m.ptr<float>(y);
                uint16_t* pu = depth_mm_u16.ptr<uint16_t>(y);
                for (int x = 0; x < depth_m.cols; ++x) {
                    float v = pd[x];
                    if (std::isfinite(v) && v > 0.0f && v < 65.535f) {
                        pu[x] = static_cast<uint16_t>(std::lround(v * 1000.0f));
                    } else {
                        pu[x] = 0;
                    }
                }
            }
            cv::imwrite(depth_filename, depth_mm_u16);

            RCLCPP_INFO(this->get_logger(),
                "💾 Saved RGB: %s | Disparity: %s | Depth(mm): %s",
                rgb_filename.c_str(), disp_filename.c_str(), depth_filename.c_str());

            frame_counter_++;
            last_saved_time_ = this->now();
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoObstacleLocalizer>());
    rclcpp::shutdown();
    return 0;
}

