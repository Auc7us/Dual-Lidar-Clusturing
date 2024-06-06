cmake_minimum_required(VERSION 3.5)
project(pcl_warping)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(lidar_warping_node src/lidar_warping_node.cpp)

ament_target_dependencies(lidar_warping_node
  rclcpp
  sensor_msgs
  PCL
  Eigen3
  pcl_conversions
)

install(TARGETS
  lidar_warping_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
