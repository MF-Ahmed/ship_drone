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

# === Generate interfaces from .action files ===
# Make sure these files are located at: crazyflie_servers/action/*.action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Explore.action"
  "action/MoveForward.action"
  "action/NavigateToHover.action"
  "action/Track.action"
  "action/TrackTarget.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs nav_msgs
)

# Export runtime so other targets can find generated headers
ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

# === Executables ===
add_executable(navigate_to_hover_server src/navigate_to_hover_server.cpp)
ament_target_dependencies(navigate_to_hover_server
  rclcpp rclcpp_action geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs std_msgs std_srvs)
rosidl_target_interfaces(navigate_to_hover_server ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(navigate_to_hover_server Threads::Threads)




add_executable(move_forward_server src/move_forward_server.cpp)
ament_target_dependencies(move_forward_server
  rclcpp rclcpp_action geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs std_msgs std_srvs)
rosidl_target_interfaces(move_forward_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(explore_and_track_controller src/explore_and_track_controller.cpp)
ament_target_dependencies(explore_and_track_controller
  rclcpp rclcpp_action geometry_msgs nav_msgs std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs)
rosidl_target_interfaces(explore_and_track_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(path_tracer src/path_tracer.cpp)
ament_target_dependencies(path_tracer
  rclcpp nav_msgs geometry_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(test_mission_client src/test_mission_client.cpp)
ament_target_dependencies(test_mission_client
  rclcpp rclcpp_action geometry_msgs nav_msgs std_msgs std_srvs)
rosidl_target_interfaces(test_mission_client ${PROJECT_NAME} "rosidl_typesupport_cpp")




 
## NEW: dual_action_client (hover -> forward) using your actions
add_executable(dual_action_client src/dual_action_client.cpp)
ament_target_dependencies(dual_action_client
  rclcpp rclcpp_action geometry_msgs nav_msgs)
rosidl_target_interfaces(dual_action_client ${PROJECT_NAME} "rosidl_typesupport_cpp")








# === Install ===
install(TARGETS
  navigate_to_hover_server
  move_forward_server
  explore_and_track_controller
  path_tracer
  test_mission_client
  dual_action_client
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
