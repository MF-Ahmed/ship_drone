cmake_minimum_required(VERSION 3.14)
project(aquabot_ekf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(/usr/include)
link_directories(/usr/lib /usr/lib/x86_64-linux-gnu)

add_executable(ekf_node src/ekf_node.cpp)

ament_target_dependencies(ekf_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ✅ Correct name of the library
target_link_libraries(ekf_node
  GeographicLib
)

install(TARGETS ekf_node
  DESTINATION lib/${PROJECT_NAME}
)






install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
