cmake_minimum_required(VERSION 3.5)
project(mtm_joint_pub)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(joint_publisher_node src/joint_publisher_node.cpp)

ament_target_dependencies(joint_publisher_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  std_msgs
  tf2_geometry_msgs
)

install(TARGETS
  joint_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
