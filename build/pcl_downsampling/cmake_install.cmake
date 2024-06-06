# Install script for directory: /home/flash/ros2_workspace/src/pcl_downsampling

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/flash/ros2_workspace/src/install/pcl_downsampling")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl_downsampling/downsampling_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl_downsampling/downsampling_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl_downsampling/downsampling_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pcl_downsampling" TYPE EXECUTABLE FILES "/home/flash/ros2_workspace/src/build/pcl_downsampling/downsampling_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl_downsampling/downsampling_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl_downsampling/downsampling_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl_downsampling/downsampling_node"
         OLD_RPATH "/home/flash/ros2_humble/install/message_filters/lib:/home/flash/ros2_humble/install/rclcpp/lib:/home/flash/ros2_humble/install/rosidl_typesupport_fastrtps_c/lib:/home/flash/ros2_humble/install/rmw/lib:/home/flash/ros2_humble/install/rosidl_typesupport_fastrtps_cpp/lib:/home/flash/ros2_humble/install/rosidl_typesupport_cpp/lib:/home/flash/ros2_humble/install/rosidl_typesupport_introspection_cpp/lib:/home/flash/ros2_humble/install/rcpputils/lib:/home/flash/ros2_humble/install/rosidl_typesupport_c/lib:/home/flash/ros2_humble/install/rosidl_typesupport_introspection_c/lib:/home/flash/ros2_humble/install/rcutils/lib:/home/flash/ros2_humble/install/rosidl_runtime_c/lib:/home/flash/ros2_humble/install/sensor_msgs/lib:/home/flash/ros2_humble/install/libstatistics_collector/lib:/home/flash/ros2_humble/install/rcl/lib:/home/flash/ros2_humble/install/rcl_interfaces/lib:/home/flash/ros2_humble/install/rcl_yaml_param_parser/lib:/home/flash/ros2_humble/install/rosgraph_msgs/lib:/home/flash/ros2_humble/install/statistics_msgs/lib:/home/flash/ros2_humble/install/tracetools/lib:/home/flash/ros2_humble/install/geometry_msgs/lib:/home/flash/ros2_humble/install/std_msgs/lib:/home/flash/ros2_humble/install/builtin_interfaces/lib:/home/flash/ros2_humble/install/rmw_implementation/lib:/home/flash/ros2_humble/install/ament_index_cpp/lib:/home/flash/ros2_humble/install/rcl_logging_spdlog/lib:/home/flash/ros2_humble/install/rcl_logging_interface/lib:/home/flash/ros2_humble/install/libyaml_vendor/lib:/home/flash/ros2_humble/install/fastcdr/lib:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/pcl_downsampling/downsampling_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/flash/ros2_workspace/src/build/pcl_downsampling/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/pcl_downsampling")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/flash/ros2_workspace/src/build/pcl_downsampling/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/pcl_downsampling")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl_downsampling/environment" TYPE FILE FILES "/home/flash/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl_downsampling/environment" TYPE FILE FILES "/home/flash/ros2_workspace/src/build/pcl_downsampling/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl_downsampling/environment" TYPE FILE FILES "/home/flash/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl_downsampling/environment" TYPE FILE FILES "/home/flash/ros2_workspace/src/build/pcl_downsampling/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl_downsampling" TYPE FILE FILES "/home/flash/ros2_workspace/src/build/pcl_downsampling/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl_downsampling" TYPE FILE FILES "/home/flash/ros2_workspace/src/build/pcl_downsampling/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl_downsampling" TYPE FILE FILES "/home/flash/ros2_workspace/src/build/pcl_downsampling/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl_downsampling" TYPE FILE FILES "/home/flash/ros2_workspace/src/build/pcl_downsampling/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl_downsampling" TYPE FILE FILES "/home/flash/ros2_workspace/src/build/pcl_downsampling/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/flash/ros2_workspace/src/build/pcl_downsampling/ament_cmake_index/share/ament_index/resource_index/packages/pcl_downsampling")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl_downsampling/cmake" TYPE FILE FILES
    "/home/flash/ros2_workspace/src/build/pcl_downsampling/ament_cmake_core/pcl_downsamplingConfig.cmake"
    "/home/flash/ros2_workspace/src/build/pcl_downsampling/ament_cmake_core/pcl_downsamplingConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl_downsampling" TYPE FILE FILES "/home/flash/ros2_workspace/src/pcl_downsampling/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/flash/ros2_workspace/src/build/pcl_downsampling/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
