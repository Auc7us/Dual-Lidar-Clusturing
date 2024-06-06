cmake_minimum_required(VERSION 3.5)
project(pcl_downsampling)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(downsampling_node src/downsampling_node.cpp)
ament_target_dependencies(downsampling_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(downsampling_node ${PCL_LIBRARIES})

install(TARGETS
  downsampling_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

