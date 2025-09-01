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


using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;





class StereoObstacleLocalizer : public rclcpp::Node
{
public:
    StereoObstacleLocalizer()
    : Node("stereo_obstacle_localizer"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        using namespace message_filters;
         // Setup save directory under current working directory        
        save_dir_ = "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/src/images";
        std::filesystem::create_directories(save_dir_ + "/rgb");
        std::filesystem::create_directories(save_dir_ + "/disparity");

        // Initialize image saving timer variables
        last_saved_time_ = this->now();
      


        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        
        // Log it for confirmation
        if (use_sim_time) {
            RCLCPP_INFO(this->get_logger(), "✅ use_sim_time is ENABLED.");
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ use_sim_time is DISABLED.");
        }


        namespace_ = this->get_namespace();
        if (namespace_ == "/drone1") {
            marker_color_r_ = 1.0; marker_color_g_ = 0.0; marker_color_b_ = 0.0; // Red
        } else if (namespace_ == "/drone2") {
            marker_color_r_ = 0.0; marker_color_g_ = 0.0; marker_color_b_ = 1.0; // Blue
        } else if (namespace_ == "/drone3") {
            marker_color_r_ = 0.0; marker_color_g_ = 1.0; marker_color_b_ = 0.0; // Green
        } else {
            marker_color_r_ = 1.0; marker_color_g_ = 1.0; marker_color_b_ = 1.0; // White fallback
        }


        left_sub_.subscribe(this, "downward_left_camera/image_raw");
        right_sub_.subscribe(this, "downward_right_camera/image_raw");
        yolo_sub_.subscribe(this, "yolo_detections");

        sync_ = std::make_shared<Synchronizer<SyncPolicy>>(SyncPolicy(100), left_sub_, right_sub_, yolo_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(1.0));
        sync_->registerCallback(std::bind(&StereoObstacleLocalizer::callback, this, _1, _2, _3));

        left_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "downward_left_camera/camera_info", 10,
            std::bind(&StereoObstacleLocalizer::leftInfoCallback, this, _1));
        right_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "downward_right_camera/camera_info", 10,
            std::bind(&StereoObstacleLocalizer::rightInfoCallback, this, _1));

        point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("obstacle_points", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", 10);

        RCLCPP_INFO(this->get_logger(), "Stereo Obstacle Localizer Node Initialized");
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image,
        vision_msgs::msg::Detection2DArray> SyncPolicy;

    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_, right_sub_;
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> yolo_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_, right_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string save_dir_;
    rclcpp::Time last_saved_time_; // Time of last saved frame
    rclcpp::Duration save_interval_{rclcpp::Duration::from_seconds(2.5)}; // save images at 5 Sec interval  


   

    int frame_counter_ = 0;

    cv::Mat K1_, D1_, K2_, D2_;
    cv::Mat R_, T_, R1_, R2_, P1_, P2_, Q_;
    cv::Mat map1x_, map1y_, map2x_, map2y_;
    bool info_ready_ = false;

    visualization_msgs::msg::MarkerArray marker_array_; // Persistent marker array

    std::string namespace_;
    double marker_color_r_, marker_color_g_, marker_color_b_;


    void leftInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        K1_ = cv::Mat(3, 3, CV_64F, (void *)msg->k.data()).clone();
        D1_ = cv::Mat(1, 5, CV_64F, (void *)msg->d.data()).clone();
        R_ = cv::Mat(3, 3, CV_64F, (void *)msg->r.data()).clone();
        RCLCPP_INFO_ONCE(this->get_logger(), "Left camera info received");
        checkCalibrationReady();
    }

    void rightInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        K2_ = cv::Mat(3, 3, CV_64F, (void *)msg->k.data()).clone();
        D2_ = cv::Mat(1, 5, CV_64F, (void *)msg->d.data()).clone();
        T_ = (cv::Mat_<double>(3, 1) << -0.10, 0, 0);  // baseline 10cm
        RCLCPP_INFO_ONCE(this->get_logger(), "Right camera info received");
        checkCalibrationReady();
    }

    void checkCalibrationReady()
    {
        if (!K1_.empty() && !K2_.empty() && !D1_.empty() && !D2_.empty() && !R_.empty())
        {
            cv::Size img_size(640, 480);  // assumed image size
            cv::stereoRectify(K1_, D1_, K2_, D2_, img_size, R_, T_, R1_, R2_, P1_, P2_, Q_);
            cv::initUndistortRectifyMap(K1_, D1_, R1_, P1_, img_size, CV_32FC1, map1x_, map1y_);
            cv::initUndistortRectifyMap(K2_, D2_, R2_, P2_, img_size, CV_32FC1, map2x_, map2y_);
            info_ready_ = true;
        }
    }

    void callback(
        const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &right_msg,
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr &dets_msg)
    {
        if (!info_ready_) return;

        cv::Mat left = cv_bridge::toCvCopy(left_msg, "bgr8")->image;
        cv::Mat right = cv_bridge::toCvCopy(right_msg, "bgr8")->image;
        

        // Rectify images
        cv::Mat left_rect, right_rect;
        cv::remap(left, left_rect, map1x_, map1y_, cv::INTER_LINEAR);
        cv::remap(right, right_rect, map2x_, map2y_, cv::INTER_LINEAR);


        // === NEW: Visualize rectified images (optional) ===
        cv::imshow("Left Rectified", left_rect);
        cv::imshow("Right Rectified", right_rect);
        cv::waitKey(1);
         // Convert to grayscale

        cv::Mat left_gray, right_gray;
        cv::cvtColor(left_rect, left_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_rect, right_gray, cv::COLOR_BGR2GRAY);

        int min_disp = -64;              // Allow negative disparities for left coverage
        int num_disp = 128;              // Range of disparities (must be multiple of 16)
        int block_size = 5;              // Block matching size

        cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        min_disp, num_disp, block_size);

        sgbm->setP1(8 * left_gray.channels() * block_size * block_size);
        sgbm->setP2(32 * left_gray.channels() * block_size * block_size);
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);  // Better accuracy
        sgbm->setSpeckleWindowSize(100);                // Suppress noise
        sgbm->setSpeckleRange(32);
        sgbm->setDisp12MaxDiff(1);                     // Allow small differences



        cv::Mat disparity_raw, disparity;
        sgbm->compute(left_gray, right_gray, disparity_raw);
        disparity_raw.convertTo(disparity, CV_32F, 1.0 / 16.0);


    // 💡 In the callback, insert this around the saving block:
     if ((this->now() - last_saved_time_) >= save_interval_) {
         // === SAVE ===
        std::string rgb_filename = save_dir_ + "/rgb/frame_" + std::to_string(frame_counter_) + ".jpg";
        std::string disp_filename = save_dir_ + "/disparity/frame_" + std::to_string(frame_counter_) + ".png";

        cv::imwrite(rgb_filename, left_rect);

        cv::Mat disp_norm;
        cv::normalize(disparity, disp_norm, 0, 255, cv::NORM_MINMAX);
        disp_norm.convertTo(disp_norm, CV_8U);
        cv::imwrite(disp_filename, disp_norm);

        RCLCPP_INFO(this->get_logger(),
            "💾 Saved RGB: %s\n💾 Saved Disparity: %s",
            rgb_filename.c_str(),
            disp_filename.c_str());
        
        frame_counter_++;      
        ///////////
        last_saved_time_ = this->now();
     }

     else {
        RCLCPP_DEBUG(this->get_logger(), "⏱ Skipping frame — saving interval not yet passed");
    }

              

        cv::Mat points3D;
        cv::reprojectImageTo3D(disparity, points3D, Q_);


        // Draw YOLO bounding boxes on disparity map
        for (const auto &det : dets_msg->detections) {
            int cx = static_cast<int>(det.bbox.center.position.x);
            int cy = static_cast<int>(det.bbox.center.position.y);
            int w = static_cast<int>(det.bbox.size_x);
            int h = static_cast<int>(det.bbox.size_y);

            cv::rectangle(disparity, cv::Rect(cx - w/2, cy - h/2, w, h), cv::Scalar(255), 2);
        }

        // Normalize and show disparity
        cv::Mat disp_vis;
        cv::normalize(disparity, disp_vis, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::imshow("Disparity with YOLO detections", disp_vis);
        cv::waitKey(1);

        int id_offset = marker_array_.markers.size(); // Offset IDs
        int id = 0;
        

        for (const auto &det : dets_msg->detections)
        {
            int cx = static_cast<int>(det.bbox.center.position.x);
            int cy = static_cast<int>(det.bbox.center.position.y);
            //RCLCPP_INFO(this->get_logger(), "Detection bbox center: x=%.2f, y=%.2f", (float)cx, (float)cy);

            if (cx >= 0 && cx < disparity.cols && cy >= 0 && cy < disparity.rows)
            {
                int cx = static_cast<int>(det.bbox.center.position.x);
                int cy = static_cast<int>(det.bbox.center.position.y);
                               

                // Sample 7x7 window
                cv::Rect roi(
                    std::max(0, cx - 3), std::max(0, cy - 3),
                    std::min(5, disparity.cols - cx + 3), std::min(5, disparity.rows - cy + 3)
                );

                cv::Mat disparity_roi = disparity(roi);
                int valid_count = cv::countNonZero(disparity_roi > 0);

                if (valid_count < (roi.area() * 0.1))
                {
                    //RCLCPP_WARN(this->get_logger(), "Too few valid disparities in ROI at (%d, %d)", cx, cy);
                    continue;
                }

                cv::Scalar mean_disp = cv::mean(disparity_roi, disparity_roi > 0);
                float disp = static_cast<float>(mean_disp[0]);

                if (disp <= 1.0f)
                {
                    //RCLCPP_WARN(this->get_logger(), "Invalid disparity after ROI filtering at (%d, %d): %.2f", cx, cy, disp);
                    continue;
                }

                cv::Vec3f point = points3D.at<cv::Vec3f>(cy, cx);

                if (!std::isfinite(point[2]))
                {
                    //RCLCPP_WARN(this->get_logger(), "Point at (%d, %d): NaN or Inf -> x=%.2f y=%.2f z=%.2f",
                                //cx, cy, point[0], point[1], point[2]);
                    continue;
                }

                if (point[2] < 0.1)
                {
                    //RCLCPP_WARN(this->get_logger(), "Point at (%d, %d): Too close -> x=%.2f y=%.2f z=%.2f",
                                //cx, cy, point[0], point[1], point[2]);
                    continue;
                }

                if (point[2] > 15.0)
                {
                    //RCLCPP_WARN(this->get_logger(), "Point at (%d, %d): Too far -> x=%.2f y=%.2f z=%.2f",
                                //cx, cy, point[0], point[1], point[2]);
                    continue;
                } 


                if (disp > 0.01)
                {
                    cv::Vec3f point = points3D.at<cv::Vec3f>(cy, cx);
                    RCLCPP_INFO(this->get_logger(), "Reprojected 3D point: x=%.2f y=%.2f z=%.2f", point[0], point[1], point[2]);

                    if (std::isfinite(point[2]) && point[2] > 0.1 && point[2] < 20.0)

                    {
                        std::string ns = this->get_namespace();
                        if (!ns.empty() && ns.front() == '/') {
                            ns.erase(0, 1);  // remove leading slash
                        }                          

                        geometry_msgs::msg::PointStamped p_cam;
                        p_cam.header = det.header;
                        p_cam.header.frame_id = ns + "/downward_left_camera_link";
                        p_cam.header.stamp = this->get_clock()->now(); 
                        p_cam.point.x = point[0];
                        p_cam.point.y = point[1];
                        p_cam.point.z = point[2];

                        try
                        {
                            geometry_msgs::msg::PointStamped p_world;
                            tf_buffer_.transform(p_cam, p_world, "world", tf2::durationFromSec(0.5));

                            RCLCPP_INFO(this->get_logger(), "Transformed point to world: x=%.2f y=%.2f z=%.2f",
                                        p_world.point.x, p_world.point.y, p_world.point.z);

                            point_pub_->publish(p_world);

                            // === SPHERE MARKER ===
                            visualization_msgs::msg::Marker marker;
                            marker.header = p_world.header;
                            marker.header.frame_id = "world";
                            marker.ns = ns +"_obstacles";
                            marker.id = id_offset+id*2;
                            marker.type = visualization_msgs::msg::Marker::SPHERE;
                            marker.action = visualization_msgs::msg::Marker::ADD;
                            marker.pose.position.x = p_world.point.x;
                            marker.pose.position.y = p_world.point.y;
                            marker.pose.position.z = 0; // clamp z
                            marker.pose.orientation.w = 1.0;
                            marker.scale.x = 0.25;
                            marker.scale.y = 0.25;
                            marker.scale.z = 0.25;
                            marker.color.a = 1.0;
                            marker.color.r = marker_color_r_;
                            marker.color.g = marker_color_g_;
                            marker.color.b = marker_color_b_;                            

                            marker.lifetime = rclcpp::Duration::from_seconds(0.0);
                            
                            marker_array_.markers.push_back(marker);

                            // === TEXT LABEL MARKER ===
                            visualization_msgs::msg::Marker text_marker = marker;
                            text_marker.id = id_offset+id * 2 + 1;
                            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                            text_marker.text = "ID: " + det.results[0].hypothesis.class_id;
                            text_marker.pose.position.z += 1.0; // offset text
                            text_marker.scale.z = 0.4;
                            text_marker.color.r = 1.0;
                            text_marker.color.g = 1.0;
                            text_marker.color.b = 1.0;
                            marker_array_.markers.push_back(text_marker);
                            id++;
                        }
                        catch (const tf2::TransformException &ex)
                        {
                            RCLCPP_WARN(this->get_logger(), "Could not transform point to world frame: %s", ex.what());
                        }
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Invalid point: z=%.2f", point[2]);
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Low or invalid disparity at (%d, %d): %.2f", cx, cy, disp);
                }
            }
        }

        marker_pub_->publish(marker_array_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoObstacleLocalizer>());
    rclcpp::shutdown();
    return 0;
}
