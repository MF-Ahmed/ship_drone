cmake_minimum_required(VERSION 3.8)
project(crazyflie_servers)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Build tools
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Runtime deps used in code and/or actions
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Threads REQUIRED)
find_package(crazyflie_yolo REQUIRED)
find_package(visualization_msgs REQUIRED)


# === Generate interfaces from .action files ===
# The .action files must be under: crazyflie_servers/action/*.action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Explore.action"
  "action/MoveForward.action"
  "action/NavigateToHover.action"
  "action/Track.action"
  "action/TrackTarget.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs nav_msgs
)

# Make the C++ typesupport target for our actions (provides include dirs/defs)
rosidl_get_typesupport_target(
  cpp_typesupport_target
  "${PROJECT_NAME}"
  "rosidl_typesupport_cpp"
)

# Export runtime so other targets can find generated headers
ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

# === Executables ===
add_executable(navigate_to_hover_server src/navigate_to_hover_server.cpp)
ament_target_dependencies(navigate_to_hover_server
  rclcpp rclcpp_action geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs std_msgs std_srvs)
target_link_libraries(navigate_to_hover_server Threads::Threads "${cpp_typesupport_target}")
add_dependencies(navigate_to_hover_server ${PROJECT_NAME}__rosidl_generator_cpp)

add_executable(move_forward_server src/move_forward_server.cpp)
ament_target_dependencies(move_forward_server
  rclcpp rclcpp_action geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs std_msgs std_srvs)
target_link_libraries(move_forward_server "${cpp_typesupport_target}")
add_dependencies(move_forward_server ${PROJECT_NAME}__rosidl_generator_cpp)

add_executable(explore_and_track_controller src/explore_and_track_controller.cpp)
ament_target_dependencies(explore_and_track_controller
  rclcpp rclcpp_action geometry_msgs nav_msgs std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs)
target_link_libraries(explore_and_track_controller "${cpp_typesupport_target}")
add_dependencies(explore_and_track_controller ${PROJECT_NAME}__rosidl_generator_cpp)

add_executable(path_tracer src/path_tracer.cpp)
ament_target_dependencies(path_tracer
  rclcpp nav_msgs geometry_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)
# (no actions used here, so no need to link typesupport)

## dual_action_client (hover -> forward) using your actions
add_executable(dual_action_client src/dual_action_client.cpp)
ament_target_dependencies(dual_action_client
  rclcpp rclcpp_action geometry_msgs nav_msgs)
target_link_libraries(dual_action_client "${cpp_typesupport_target}")
add_dependencies(dual_action_client ${PROJECT_NAME}__rosidl_generator_cpp)

# Surveillance patrol client (uses actions + crazyflie_yolo msgs)
add_executable(surveillance_patrol_client src/surveillance_patrol_client.cpp)
ament_target_dependencies(surveillance_patrol_client
  rclcpp rclcpp_action geometry_msgs nav_msgs std_msgs crazyflie_yolo)
target_link_libraries(surveillance_patrol_client "${cpp_typesupport_target}")
add_dependencies(surveillance_patrol_client ${PROJECT_NAME}__rosidl_generator_cpp)


add_executable(assigned_goal_dispatcher src/assigned_goal_dispatcher.cpp)
ament_target_dependencies(assigned_goal_dispatcher
  rclcpp rclcpp_action geometry_msgs crazyflie_yolo)
rosidl_target_interfaces(assigned_goal_dispatcher ${PROJECT_NAME} "rosidl_typesupport_cpp")





add_executable(dopt_tracking_metrics src/dopt_tracking_metrics.cpp)
ament_target_dependencies(dopt_tracking_metrics
  rclcpp geometry_msgs nav_msgs std_msgs visualization_msgs crazyflie_yolo)


add_executable(qstar_dual_action_client src/qstar_dual_action_client.cpp)
ament_target_dependencies(qstar_dual_action_client
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(qstar_dual_action_client "${cpp_typesupport_target}")
add_dependencies(qstar_dual_action_client ${PROJECT_NAME}__rosidl_generator_cpp)


# === Install ===
install(TARGETS
  navigate_to_hover_server
  move_forward_server
  explore_and_track_controller
  path_tracer
  surveillance_patrol_client
  dual_action_client
  assigned_goal_dispatcher
  dopt_tracking_metrics
  qstar_dual_action_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# (Optional) install the .action sources alongside the package
install(DIRECTORY action/
  DESTINATION share/${PROJECT_NAME}/action
)

ament_package()
