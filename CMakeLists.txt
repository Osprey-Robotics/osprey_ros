################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(osprey_robotics)

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
)

foreach(Dependency IN ITEMS ${INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

pkg_search_module(libusb-1.0 REQUIRED libusb-1.0)

################################################################################
# Build
################################################################################
include_directories(include)

add_library(robot_hardware_interface
  SHARED
  src/usb.cpp
  src/joint.cpp
  src/robot.cpp
)

add_executable(test-usb-motors
  src/test-usb-motors.cpp
)

target_link_libraries(test-usb-motors robot_hardware_interface usb-1.0)