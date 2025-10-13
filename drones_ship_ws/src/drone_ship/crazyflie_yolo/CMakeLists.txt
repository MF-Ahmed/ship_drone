cmake_minimum_required(VERSION 3.8)
project(crazyflie_yolo)

# =========================
# Compiler warnings
# =========================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Find packages
# =========================
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_geometry REQUIRED)

find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

enable_language(CUDA)

# =========================
# Include directories
# =========================
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  # generated headers during build
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Optional external libs (your SGM install)
include_directories(/usr/local/include)
link_directories(/usr/local/lib)
link_directories(${CUDA_TOOLKIT_ROOT_DIR}/lib64)

# =========================
# Interfaces (msgs/actions)
# =========================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedObstacle.msg"
  "msg/TrackedObstacleArray.msg"
  "msg/Detection3DStamped.msg"
  "msg/AssignTarget.msg"
  "action/Explore.action"
  "action/MoveForward.action"
  "action/NavigateToHover.action"
  "action/Track.action"
  "action/TrackTarget.action"
  DEPENDENCIES std_msgs geometry_msgs vision_msgs action_msgs
  # Remove this once your environment stops tripping the install guard:
  SKIP_INSTALL
)

# Make generated interfaces available to downstream users of this pkg
ament_export_dependencies(rosidl_default_runtime)

# =========================
# Executables
# =========================

# -- Stereo depth node
add_executable(stereo_depth_node src/stereo_depth_node.cpp)
ament_target_dependencies(stereo_depth_node
  rclcpp
  sensor_msgs
  std_msgs
  vision_msgs
  cv_bridge
  image_transport
  visualization_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
  OpenCV
  image_geometry
  nav_msgs
)
target_link_libraries(stereo_depth_node
  sgm
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
)
add_dependencies(stereo_depth_node ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_target_interfaces(stereo_depth_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# -- Multi obstacle tracker (non-EKF)
add_executable(multi_obstacle_tracker_node src/multi_obstacle_tracker.cpp)
ament_target_dependencies(multi_obstacle_tracker_node
  rclcpp
  geometry_msgs
  visualization_msgs
  Eigen3
  tf2
  tf2_ros
  tf2_geometry_msgs
)
add_dependencies(multi_obstacle_tracker_node ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_target_interfaces(multi_obstacle_tracker_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# -- Multi-drone fusion node
add_executable(multi_drone_fusion_node src/multi_drone_fusion_node.cpp)
ament_target_dependencies(multi_drone_fusion_node
  rclcpp
  geometry_msgs
  visualization_msgs
  Eigen3
  tf2
  tf2_ros
  tf2_geometry_msgs
)
add_dependencies(multi_drone_fusion_node ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_target_interfaces(multi_drone_fusion_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# -- Multi obstacle tracker EKF node
add_executable(multi_obstacle_tracker_ekf_node src/multi_obstacle_tracker_EKF.cpp)
ament_target_dependencies(multi_obstacle_tracker_ekf_node
  rclcpp
  geometry_msgs
  visualization_msgs
  Eigen3
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  nav_msgs
)
add_dependencies(multi_obstacle_tracker_ekf_node ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_target_interfaces(multi_obstacle_tracker_ekf_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# -- Assignment nodes (Hungarian)
add_executable(assignment_node src/assignment_node.cpp)
ament_target_dependencies(assignment_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  Eigen3
)
add_dependencies(assignment_node ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_target_interfaces(assignment_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# =========================
# Install C++ binaries
# =========================
install(TARGETS
  stereo_depth_node
  multi_obstacle_tracker_node
  multi_drone_fusion_node
  multi_obstacle_tracker_ekf_node
  assignment_node
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# Python scripts & Launch
# =========================
install(PROGRAMS
  scripts/yolo_detector_node.py
  scripts/crazyflie_segmentation_yolo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# =========================
# Testing (optional)
# =========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
