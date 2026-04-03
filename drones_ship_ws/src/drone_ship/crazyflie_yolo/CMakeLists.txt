cmake_minimum_required(VERSION 3.16)
project(crazyflie_yolo)

# =========================
# C++ standard & warnings
# =========================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# ROS 2 dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)    # for interface generation
find_package(Eigen3 REQUIRED)

# =========================
# Third-party dependencies
# =========================
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# =========================
# Includes
# =========================
include_directories(include)

# =========================
# Interfaces (msgs) in this package
# =========================
set(msg_files
  "msg/AssignTarget.msg"
  "msg/Detection3DStamped.msg"
  "msg/TrackedObstacle.msg"
  "msg/TrackedObstacleArray.msg"
  "msg/AssignTargetArray.msg"
  "msg/AssignmentMetrics.msg"
  
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Get the C++ typesupport target for linking/ordering with our nodes
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# =========================
# Helper to declare nodes
# =========================
# Usage: make_node(<target_name> <src_file> <installed_name>)
function(make_node target_name src_file installed_name)
  add_executable(${target_name} ${src_file})

  # Ensure generated interfaces are built first and linked (modern approach)
  add_dependencies(${target_name} ${cpp_typesupport_target})
  target_link_libraries(${target_name} ${cpp_typesupport_target})

  # Name the installed binary exactly as launch expects
  set_target_properties(${target_name} PROPERTIES OUTPUT_NAME ${installed_name})

  target_include_directories(${target_name} PRIVATE include)
  ament_target_dependencies(${target_name}
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    message_filters
    image_transport
    cv_bridge
    stereo_msgs 
    vision_msgs
  )
  target_link_libraries(${target_name}
    Eigen3::Eigen
    ${OpenCV_LIBS}
  )

  install(TARGETS ${target_name}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

# =========================
# Executables (installed names match launch)
# =========================
make_node(assignment_node            src/assignment_node.cpp            assignment_node)
make_node(multi_drone_fusion_node    src/multi_drone_fusion_node.cpp    multi_drone_fusion_node)
#make_node(multi_obstacle_tracker     src/multi_obstacle_tracker.cpp     multi_obstacle_tracker_node)
make_node(multi_obstacle_tracker_ekf src/multi_obstacle_tracker_EKF.cpp multi_obstacle_tracker_ekf_node)
make_node(stereo_depth_node          src/stereo_depth_node.cpp          stereo_depth_node)


add_executable(qstar_goal_node src/qstar_goal_node.cpp)
ament_target_dependencies(qstar_goal_node
  rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs 
)

target_link_libraries(qstar_goal_node Eigen3::Eigen)



install(TARGETS qstar_goal_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_target_dependencies(assignment_node
  rclcpp nav_msgs geometry_msgs visualization_msgs std_msgs
  tf2 tf2_geometry_msgs
)

make_node(ab3dmot_3d_tracker      src/ab3dmot_3d_tracker_node.cpp      ab3dmot_3d_tracker_node)
make_node(sort_tracker            src/sort_tracker_node.cpp            sort_tracker_node)
make_node(tracking_metrics        src/tracking_metrics_node.cpp        tracking_metrics_node)


make_node(ftci_fusion          src/ftci_fusion_node.cpp          ftci_fusion_node)
make_node(fusion_metrics       src/fusion_metrics_node.cpp       fusion_metrics_node)



# =========================
# Python nodes / scripts
# =========================
install(PROGRAMS
  scripts/yolo_detector_node.py
  scripts/dopt_plotter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# Install non-code assets
# =========================
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# =========================
# Lint / tests
# =========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
