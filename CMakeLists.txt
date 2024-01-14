################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(osprey_ros)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-g -Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
set(INCLUDE_DEPENDS
  PkgConfig
  hardware_interface
  # control_msgs
  # std_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  # controller_interface
  controller_manager
  joint_limits
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

pkg_search_module(libusb-1.0 REQUIRED libusb-1.0)

################################################################################
# Build
################################################################################
if(BUILD_TESTING)
 find_package(ament_lint_auto REQUIRED)
 ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_library(robot_hardware_interface
  SHARED
  src/usb.cpp
  src/joint.cpp
  src/robot.cpp
  src/robot_hardware_interface.cpp
)

add_executable(test-usb-motors
  src/test-usb-motors.cpp
)

ament_target_dependencies(robot_hardware_interface ${INCLUDE_DEPENDS})

target_link_libraries(robot_hardware_interface usb-1.0)
target_link_libraries(test-usb-motors robot_hardware_interface usb-1.0)

################################################################################
# Plugin
################################################################################
pluginlib_export_plugin_description_file(hardware_interface plugins/robot_hardware_interface.xml)

################################################################################
# Install
################################################################################
install(
 DIRECTORY config description launch2 meshes worlds
 DESTINATION share/${PROJECT_NAME}
)

install(TARGETS robot_hardware_interface
 EXPORT robot_hardware_interface
 ARCHIVE DESTINATION lib
 LIBRARY DESTINATION lib
 RUNTIME DESTINATION bin
)
################################################################################
# Macro for ament package
################################################################################
ament_package()
