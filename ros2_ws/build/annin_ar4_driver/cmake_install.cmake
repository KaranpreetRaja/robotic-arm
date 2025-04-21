# Install script for directory: /home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/ar4_ros_driver/annin_ar4_driver

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/install/annin_ar4_driver")
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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/annin_ar4_driver/run_arduino_nano" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/annin_ar4_driver/run_arduino_nano")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/annin_ar4_driver/run_arduino_nano"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/annin_ar4_driver" TYPE EXECUTABLE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/run_arduino_nano")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/annin_ar4_driver/run_arduino_nano" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/annin_ar4_driver/run_arduino_nano")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/annin_ar4_driver/run_arduino_nano"
         OLD_RPATH "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver:/home/abdi/ros2_humble/install/rclcpp_lifecycle/lib:/home/abdi/ros2_humble/install/ament_index_cpp/lib:/home/abdi/ros2_humble/install/sensor_msgs/lib:/home/abdi/ros2_humble/install/rclcpp/lib:/home/abdi/ros2_humble/install/rcl_action/lib:/home/abdi/ros2_humble/install/rclcpp_action/lib:/home/abdi/ros2_humble/install/libstatistics_collector/lib:/home/abdi/ros2_humble/install/rosgraph_msgs/lib:/home/abdi/ros2_humble/install/statistics_msgs/lib:/home/abdi/ros2_humble/install/builtin_interfaces/lib:/home/abdi/ros2_humble/install/geometry_msgs/lib:/home/abdi/ros2_humble/install/std_msgs/lib:/home/abdi/ros2_humble/install/trajectory_msgs/lib:/home/abdi/ros2_humble/install/action_msgs/lib:/home/abdi/ros2_humble/install/unique_identifier_msgs/lib:/home/abdi/ros2_humble/install/rosidl_typesupport_fastrtps_c/lib:/home/abdi/ros2_humble/install/rmw/lib:/home/abdi/ros2_humble/install/rosidl_typesupport_fastrtps_cpp/lib:/home/abdi/ros2_humble/install/rosidl_typesupport_cpp/lib:/home/abdi/ros2_humble/install/rosidl_typesupport_introspection_cpp/lib:/home/abdi/ros2_humble/install/rosidl_typesupport_c/lib:/home/abdi/ros2_humble/install/rosidl_typesupport_introspection_c/lib:/home/abdi/ros2_humble/install/class_loader/lib:/home/abdi/ros2_humble/install/rcl/lib:/home/abdi/ros2_humble/install/rosidl_runtime_c/lib:/home/abdi/ros2_humble/install/tracetools/lib:/home/abdi/ros2_humble/install/rcl_lifecycle/lib:/home/abdi/ros2_humble/install/lifecycle_msgs/lib:/home/abdi/ros2_humble/install/rcl_interfaces/lib:/home/abdi/ros2_humble/install/rcl_yaml_param_parser/lib:/home/abdi/ros2_humble/install/libyaml_vendor/lib:/home/abdi/ros2_humble/install/rmw_implementation/lib:/home/abdi/ros2_humble/install/rcl_logging_spdlog/lib:/home/abdi/ros2_humble/install/rcl_logging_interface/lib:/home/abdi/ros2_humble/install/fastcdr/lib:/home/abdi/ros2_humble/install/rcpputils/lib:/home/abdi/ros2_humble/install/rcutils/lib:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/annin_ar4_driver/run_arduino_nano")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/ar4_ros_driver/annin_ar4_driver/hardware_interface_plugin.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libannin_ar4_driver.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libannin_ar4_driver.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libannin_ar4_driver.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/libannin_ar4_driver.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libannin_ar4_driver.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libannin_ar4_driver.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libannin_ar4_driver.so"
         OLD_RPATH "/home/abdi/ros2_humble/install/rclcpp_lifecycle/lib:/home/abdi/ros2_humble/install/ament_index_cpp/lib:/home/abdi/ros2_humble/install/sensor_msgs/lib:/home/abdi/ros2_humble/install/rclcpp/lib:/home/abdi/ros2_humble/install/rcl_action/lib:/home/abdi/ros2_humble/install/rclcpp_action/lib:/home/abdi/ros2_humble/install/builtin_interfaces/lib:/home/abdi/ros2_humble/install/geometry_msgs/lib:/home/abdi/ros2_humble/install/std_msgs/lib:/home/abdi/ros2_humble/install/trajectory_msgs/lib:/home/abdi/ros2_humble/install/action_msgs/lib:/home/abdi/ros2_humble/install/rosidl_typesupport_fastrtps_c/lib:/home/abdi/ros2_humble/install/rmw/lib:/home/abdi/ros2_humble/install/rosidl_typesupport_fastrtps_cpp/lib:/home/abdi/ros2_humble/install/rosidl_typesupport_cpp/lib:/home/abdi/ros2_humble/install/rosidl_typesupport_introspection_cpp/lib:/home/abdi/ros2_humble/install/rosidl_typesupport_c/lib:/home/abdi/ros2_humble/install/rosidl_typesupport_introspection_c/lib:/home/abdi/ros2_humble/install/class_loader/lib:/home/abdi/ros2_humble/install/rcl/lib:/home/abdi/ros2_humble/install/rosidl_runtime_c/lib:/home/abdi/ros2_humble/install/tracetools/lib:/home/abdi/ros2_humble/install/rcl_lifecycle/lib:/home/abdi/ros2_humble/install/lifecycle_msgs/lib:/home/abdi/ros2_humble/install/rcpputils/lib:/home/abdi/ros2_humble/install/rcutils/lib:/home/abdi/ros2_humble/install/libstatistics_collector/lib:/home/abdi/ros2_humble/install/rosgraph_msgs/lib:/home/abdi/ros2_humble/install/statistics_msgs/lib:/home/abdi/ros2_humble/install/rcl_interfaces/lib:/home/abdi/ros2_humble/install/rcl_yaml_param_parser/lib:/home/abdi/ros2_humble/install/libyaml_vendor/lib:/home/abdi/ros2_humble/install/rmw_implementation/lib:/home/abdi/ros2_humble/install/rcl_logging_spdlog/lib:/home/abdi/ros2_humble/install/rcl_logging_interface/lib:/home/abdi/ros2_humble/install/unique_identifier_msgs/lib:/home/abdi/ros2_humble/install/fastcdr/lib:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libannin_ar4_driver.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/annin_ar4_driver" TYPE DIRECTORY FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/ar4_ros_driver/annin_ar4_driver/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver" TYPE DIRECTORY FILES
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/ar4_ros_driver/annin_ar4_driver/config"
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/ar4_ros_driver/annin_ar4_driver/launch"
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/ar4_ros_driver/annin_ar4_driver/urdf"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/annin_ar4_driver" TYPE PROGRAM FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/ar4_ros_driver/annin_ar4_driver/scripts/reset_estop.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver/environment" TYPE FILE FILES "/home/abdi/ros2_humble/build/ament_package/ament_package/template/environment_hook/library_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver/environment" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/ament_cmake_environment_hooks/library_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/annin_ar4_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/annin_ar4_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver/environment" TYPE FILE FILES "/home/abdi/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver/environment" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver/environment" TYPE FILE FILES "/home/abdi/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver/environment" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/ament_cmake_index/share/ament_index/resource_index/packages/annin_ar4_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/hardware_interface__pluginlib__plugin" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/ament_cmake_index/share/ament_index/resource_index/hardware_interface__pluginlib__plugin/annin_ar4_driver")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver/cmake/export_annin_ar4_driverExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver/cmake/export_annin_ar4_driverExport.cmake"
         "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/CMakeFiles/Export/share/annin_ar4_driver/cmake/export_annin_ar4_driverExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver/cmake/export_annin_ar4_driverExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver/cmake/export_annin_ar4_driverExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver/cmake" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/CMakeFiles/Export/share/annin_ar4_driver/cmake/export_annin_ar4_driverExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver/cmake" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/CMakeFiles/Export/share/annin_ar4_driver/cmake/export_annin_ar4_driverExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver/cmake" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver/cmake" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver/cmake" TYPE FILE FILES
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/ament_cmake_core/annin_ar4_driverConfig.cmake"
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/ament_cmake_core/annin_ar4_driverConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/annin_ar4_driver" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/ar4_ros_driver/annin_ar4_driver/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/annin_ar4_driver/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
