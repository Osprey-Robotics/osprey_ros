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
find_package(ignition-gazebo6 QUIET)

set(INCLUDE_DEPENDS
  PkgConfig
  ament_cmake
  hardware_interface
  control_msgs
  map_msgs
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

if( ignition-gazebo6_FOUND )
    set(INCLUDE_DEPENDS
        ${INCLUDE_DEPENDS}
        ignition-cmake2
        ignition-common4
        ignition-gazebo6
        ignition-math6
        ignition-msgs8
        ignition-physics5
        ignition-plugin1
        ignition-transport11
        ignition-utils1
        sdformat12
    )
endif()

foreach(Dependency IN ITEMS ${INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

if( ignition-gazebo6_FOUND )
    pkg_search_module(ignition-cmake2 REQUIRED ignition-cmake2)
endif()
pkg_search_module(libusb-1.0 REQUIRED libusb-1.0)

################################################################################
# Build
################################################################################
if(BUILD_TESTING)
 find_package(ament_lint_auto REQUIRED)
 ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

if( ignition-gazebo6_FOUND )
    include_directories(
        ${IGNITION-CMAKE_DIR}
        /usr/include/ignition/cmake2/
        ${IGNITION-COMMON_INCLUDE_DIRS}
        ${IGNITION-GAZEBO_INCLUDE_DIRS}
        ${IGNITION-MATH_INCLUDE_DIRS}
        ${IGNITION-MSGS_INCLUDE_DIRS}
        ${IGNITION-PHYSICS_INCLUDE_DIRS}
        ${IGNITION-PLUGIN_INCLUDE_DIRS}
        ${IGNITION-TRANSPORT_INCLUDE_DIRS}
        ${IGNITION-UTILS_INCLUDE_DIRS}
        ${SDFormat_INCLUDE_DIRS}
    )

    link_directories(${IGNITION-CMAKE_LIBRARY_DIRS}
        ${IGNITION-COMMON_LIBRARY_DIRS}
        ${IGNITION-GAZEBO_LIBRARY_DIRS}
        ${IGNITION-MATH_LIBRARY_DIRS}
        ${IGNITION-MSGS_LIBRARY_DIRS}
        ${IGNITION-PLUGIN_LIBRARY_DIRS}
        ${IGNITION-PHYSICS_LIBRARY_DIRS}
        ${IGNITION-TRANSPORT_LIBRARY_DIRS}
        ${IGNITION-UTILS_LIBRARY_DIRS}
        ${SDFormat_LIBRARY_DIRS}
    )
endif()

# shared object libraries, lib*.so
add_library(autonomy SHARED src/autonomy.cpp)
add_library(gpio SHARED src/gpio.cpp)
add_library(gpio_controller SHARED src/gpio_controller.cpp)
if( ignition-gazebo6_FOUND )
    add_library(gz_random_rocks SHARED src/gz_random_rocks.cpp)
endif()
add_library(remapper SHARED src/remapper.cpp)
add_library(robot-core
  SHARED
  src/usb.cpp
  src/joint.cpp
)
add_library(robot SHARED src/robot.cpp)
add_library(robot24 SHARED  src/robot24.cpp)
add_library(robot_hardware_interface SHARED src/robot_hardware_interface.cpp)
add_library(robot24_hardware_interface SHARED src/robot24_hardware_interface.cpp)
add_library(teleop_manual_joy SHARED src/teleop_manual_joy.cpp)

# executable binaries
add_executable(autonomy_node src/autonomy_node.cpp)
add_executable(remapper_node src/remapper_node.cpp)
add_executable(teleop_manual_joy_node src/teleop_manual_joy_node.cpp)
add_executable(test-gpio-limit-switches src/tests/test-gpio-limit-switches.cpp)
add_executable(test-gpio-relays src/tests/test-gpio-relays.cpp)
add_executable(test-usb-motors24 src/tests/test-usb-motors24.cpp)
add_executable(test-usb-motors src/test-usb-motors.cpp)

# add ROS dependencies
ament_target_dependencies(gpio_controller ${INCLUDE_DEPENDS})
ament_target_dependencies(remapper ${INCLUDE_DEPENDS})
ament_target_dependencies(robot_hardware_interface ${INCLUDE_DEPENDS})
ament_target_dependencies(robot24_hardware_interface ${INCLUDE_DEPENDS})
ament_target_dependencies(autonomy ${INCLUDE_DEPENDS})
ament_target_dependencies(teleop_manual_joy ${INCLUDE_DEPENDS})

# link executable binaries to shared object libraries
target_link_libraries(autonomy_node autonomy)
target_link_libraries(remapper_node remapper)
target_link_libraries(robot-core usb-1.0)
target_link_libraries(robot robot-core gpio)
target_link_libraries(robot_hardware_interface gpio robot usb-1.0)
target_link_libraries(robot24 robot-core)
target_link_libraries(robot24_hardware_interface gpio robot24 usb-1.0)
target_link_libraries(teleop_manual_joy_node teleop_manual_joy)
target_link_libraries(test-gpio-limit-switches gpio robot)
target_link_libraries(test-gpio-relays gpio robot)
target_link_libraries(test-usb-motors robot usb-1.0)
target_link_libraries(test-usb-motors24 robot usb-1.0)

# register ROS nodes
rclcpp_components_register_nodes(autonomy "osprey_ros::Autonomy")
rclcpp_components_register_nodes(autonomy "osprey_ros::ReMapper")
rclcpp_components_register_nodes(teleop_manual_joy "teleop_manual_joy::TeleopManualJoy")

################################################################################
# Plugin
################################################################################
pluginlib_export_plugin_description_file(controller_interface plugins/gpio_controller.xml)
pluginlib_export_plugin_description_file(hardware_interface plugins/robot_hardware_interface.xml)
pluginlib_export_plugin_description_file(hardware_interface plugins/robot24_hardware_interface.xml)

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
  remapper
  remapper_node
  robot-core
  robot
  robot_hardware_interface
  robot24
  robot24_hardware_interface
  teleop_manual_joy
  teleop_manual_joy_node
)

if( ignition-gazebo6_FOUND )
    set(INSTALL_TARGETS
        ${INSTALL_TARGETS}
        gz_random_rocks
    )
endif()

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
