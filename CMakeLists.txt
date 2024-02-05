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
  control_msgs
  sensor_msgs
  std_msgs
  pluginlib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  rosidl_runtime_cpp
  controller_interface
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

# shared object libraries, lib*.so
add_library(autonomy SHARED src/autonomy.cpp)
add_library(gpio SHARED src/gpio.cpp)
add_library(gpio_controller SHARED src/gpio_controller.cpp)
add_library(robot
  SHARED
  src/usb.cpp
  src/joint.cpp
  src/robot.cpp
)
add_library(robot_hardware_interface SHARED src/robot_hardware_interface.cpp)
add_library(teleop_manual_joy SHARED src/teleop_manual_joy.cpp)

# executable binaries
add_executable(autonomy_node src/autonomy_node.cpp)
add_executable(teleop_manual_joy_node src/teleop_manual_joy_node.cpp)
add_executable(test-gpio-limit-switches src/tests/test-gpio-limit-switches.cpp)
add_executable(test-gpio-relays src/tests/test-gpio-relays.cpp)
add_executable(test-usb-motors src/test-usb-motors.cpp)

# add ROS dependencies
ament_target_dependencies(gpio_controller ${INCLUDE_DEPENDS})
ament_target_dependencies(robot_hardware_interface ${INCLUDE_DEPENDS})
ament_target_dependencies(autonomy ${INCLUDE_DEPENDS})
ament_target_dependencies(teleop_manual_joy ${INCLUDE_DEPENDS})

# link executable binaries to shared object libraries
target_link_libraries(autonomy_node autonomy)
target_link_libraries(robot gpio)
target_link_libraries(robot_hardware_interface gpio robot usb-1.0)
target_link_libraries(teleop_manual_joy_node teleop_manual_joy)
target_link_libraries(test-gpio-limit-switches gpio robot)
target_link_libraries(test-gpio-relays gpio robot)
target_link_libraries(test-usb-motors robot usb-1.0)

# register ROS nodes
rclcpp_components_register_nodes(autonomy "osprey_ros::Autonomy")
rclcpp_components_register_nodes(teleop_manual_joy "teleop_manual_joy::TeleopManualJoy")

################################################################################
# Plugin
################################################################################
pluginlib_export_plugin_description_file(controller_interface plugins/gpio_controller.xml)
pluginlib_export_plugin_description_file(hardware_interface plugins/robot_hardware_interface.xml)

################################################################################
# Install
################################################################################
install(
 DIRECTORY config description launch2 meshes worlds
 DESTINATION share/${PROJECT_NAME}
)

set(INSTALL_TARGETS
  autonomy
  autonomy_node
  gpio
  gpio_controller
  robot
  robot_hardware_interface
  teleop_manual_joy
  teleop_manual_joy_node
)

foreach(target IN ITEMS ${INSTALL_TARGETS})
  install(TARGETS ${target}
    EXPORT ${target}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

################################################################################
# Macro for ament package
################################################################################
ament_package()
