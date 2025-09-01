cmake_minimum_required(VERSION 3.8)
project(crazyflie_yolo)

# Enable warnings for C++
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Find required packages ===
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)



enable_language(CUDA)








# === Include directories ===
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)



rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedObstacle.msg"
  "msg/TrackedObstacleArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)




link_directories(${CUDA_TOOLKIT_ROOT_DIR}/lib64)


# === Add libSGM manually ===
include_directories(/usr/local/include)
link_directories(/usr/local/lib)


# === Python script install ===
install(PROGRAMS
  scripts/yolo_detector_node.py
  scripts/crazyflie_segmentation_yolo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# === Install launch files ===
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# === C++ executable: stereo_depth_node ===
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
)


target_link_libraries(stereo_depth_node
  sgm
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
)


# === C++ executable: multi_obstacle_tracker_node ===
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


rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(multi_obstacle_tracker_node "${cpp_typesupport_target}")

target_include_directories(multi_obstacle_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)




# === C++ executable: container_localizer_node ===
add_executable(container_localizer_node src/container_localizer_node.cpp)
ament_target_dependencies(container_localizer_node
  rclcpp
  sensor_msgs
  std_msgs
  vision_msgs
  cv_bridge
  image_transport
  OpenCV
  tf2_geometry_msgs
)

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

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(multi_drone_fusion_node "${cpp_typesupport_target}")



# === C++ executable: multi_obstacle_tracker_ekf_node ===
add_executable(multi_obstacle_tracker_ekf_node src/multi_obstacle_tracker_EKF.cpp)
ament_target_dependencies(multi_obstacle_tracker_ekf_node
  rclcpp
  geometry_msgs
  visualization_msgs
  Eigen3
  tf2
  tf2_ros
  tf2_geometry_msgs
)



rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(multi_obstacle_tracker_ekf_node "${cpp_typesupport_target}")

target_include_directories(multi_obstacle_tracker_ekf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)


target_include_directories(multi_drone_fusion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

# === Install C++ binaries ===
install(TARGETS
  stereo_depth_node
  container_localizer_node
  multi_obstacle_tracker_node
  multi_drone_fusion_node 
  multi_obstacle_tracker_ekf_node
  DESTINATION lib/${PROJECT_NAME}
)

# === Testing (optional) ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

