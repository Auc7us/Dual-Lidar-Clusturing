cmake_minimum_required(VERSION 3.5)
project(pcl_clustering)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(clustering_node src/clustering_node.cpp)
ament_target_dependencies(clustering_node rclcpp sensor_msgs pcl_conversions visualization_msgs)
target_link_libraries(clustering_node ${PCL_LIBRARIES})

install(TARGETS
  clustering_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
