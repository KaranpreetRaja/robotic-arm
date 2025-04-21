# Install script for directory: /home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/install/arm_control")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_control/environment" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/arm_control/ament_cmake_environment_hooks/resource_paths.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_control/description" TYPE DIRECTORY FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/description/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_control/env-hooks" TYPE DIRECTORY FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/env-hooks/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_control/environment" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/arm_control/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_control/environment" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/arm_control/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/arm_control-0.0.0-py3.10.egg-info" TYPE DIRECTORY FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/arm_control/ament_cmake_python/arm_control/arm_control.egg-info/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/arm_control" TYPE DIRECTORY FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/arm_control/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3.10" "-m" "compileall"
        "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/install/arm_control/local/lib/python3.10/dist-packages/arm_control"
      )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/arm_control" TYPE PROGRAM FILES
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/scripts/PID.py"
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/scripts/kinematics.py"
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/scripts/arm_states_encoder.py"
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/scripts/arm_states_decoder.py"
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/scripts/base_station_encoder.py"
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/scripts/base_station_decoder.py"
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/networking/base_station_transmitter.py"
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/networking/base_station_receiver.py"
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/networking/arm_receiver.py"
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/networking/arm_transmitter.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_control/" TYPE DIRECTORY FILES
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/config"
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/description"
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/launch"
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/worlds"
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/env-hooks"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_control/description" TYPE DIRECTORY FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/description/mesh" FILES_MATCHING REGEX "/[^/]*\\.STL$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/arm_control/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/arm_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/arm_control/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/arm_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_control/environment" TYPE FILE FILES "/home/abdi/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_control/environment" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/arm_control/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_control/environment" TYPE FILE FILES "/home/abdi/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_control/environment" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/arm_control/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_control" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/arm_control/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_control" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/arm_control/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_control" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/arm_control/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_control" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/arm_control/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_control" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/arm_control/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/arm_control/ament_cmake_index/share/ament_index/resource_index/packages/arm_control")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_control/cmake" TYPE FILE FILES
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/arm_control/ament_cmake_core/arm_controlConfig.cmake"
    "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/arm_control/ament_cmake_core/arm_controlConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_control" TYPE FILE FILES "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/src/arm_control/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/abdi/Documents/GitHub/robotic-arm/ros2_ws/build/arm_control/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
