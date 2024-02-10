# Osprey ROS

This project is the University of North Florida's Osprey Robotics Club ROS 
implementation for their NASA Lunabotics Remote Mining Competition Robots,
past, present, and future.

## Environment Preparation
The following assumes you have installed all the necessary ROS 2 Iron packages, and have sourced the installation before running any `ros2` commands.
```bash
source /opt/ros/iron/setup.bash
```

You may want to have your development user environment do this on login via `~/.bashrc` file; add that command to the end of that file.

Its recommended to have a ros2 workspace in a user directory for development purposes, to build this project, etc; ex `~/ros2_ws/`. The following will refer to that directory, and directories created within.

## Download
Download and unpack or clone this repositories contents into your ros2 workspace; ex `~/ros2_ws/src/osprey_ros`.


## Build and Install
Building is done using colcon which will invoke cmake and run the necessary commands. Run the following command in your ros2 workspace; ex `~/ros2_ws/`.
```bash
colcon build --symlink-install --packages-select  osprey_ros
```

### Source install
Make sure to run the following command after install and login. Run the following command in your ros2 workspace; ex `~/ros2_ws/`.
```bash
source install/setup.bash
```

You may want to have your development user environment do this on login via `~/.bashrc` file; add the following to the end of that file.
```bash
source ~/ros2_ws/install/setup.bash
```

## Docker Development Environment
Two docker development environments are available for portable development and
simulation. This requires docker to be installed on either Linux or Windows; Windows is untested. Of the two options, most will use the second option, the first will primarily be used within this repository for CI. Instructions for usage is
provide within each of the respective repositories readme.

### docker-ros2-iron image
The first environment is the [docker-ros2-iron](https://github.com/Osprey-Robotics/docker-ros2-iron) image, which contains the base headless environment and
minimum packages needed to build and run `opsrey_ros` package.

### docker-ros2-iron-gz-rviz2 image
The first environment is the [docker-ros2-iron-gz-rviz2](https://github.com/Osprey-Robotics/docker-ros2-iron-gz-rviz2) image, which uses and builds on the
previous base image with packages for graphical simulation, gazebo and rviz2.
This is the primary image most will want to use for simulation testing of the
robot and/or sensors.

## Run
To run on the actual hardware run the following command in your ros2 workspace; ex `~/ros2_ws/`.
```bash
ros2 launch osprey_ros osprey_ros.launch.py
```

## Robot Human Controllers
There are presently two ways to control the robot using [teleop twist joy](https://github.com/ros2/teleop_twist_joy) and [keyboard](https://github.com/ros2/teleop_twist_keyboard)

### Remote operation
For remote operation over IP run the following commands on both systems prior
to running ros launch files.
```bash
export ROS_MASTER_URI=<robot_ip>:12345
export ROS_IP=<robot_ip>10.1.2.7
```

### Gamepad
Run the following command to invoke the teleop manual joy controller for the gamepad. 
```bash
ros2 launch osprey_ros gamepad.launch.py
```

#### Button Layout
Movement is done using the D-Pad and the X and Right Trigger button combinations. Press either X or Right Trigger, and then use the D-Pad to move forward, backward, and turn left and right.
![Picture of Logitech F310](https://gm0.org/en/latest/_images/logitech-f310.png)

### Keyboard
Run the following command to invoke the controller for the keyboard, which will present a interface for controlling the robot in the same terminal the command is run within.
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Credits
Credits and thanks for resources used in this repository, some code and/or project structure, go to the following:

- Articulated Robotics - 
  [Making a Mobile Robot with ROS](https://articulatedrobotics.xyz/mobile-robot-full-list/)
- ROS 2 Control Demos -
  [example 2](https://github.com/ros-controls/ros2_control_demos)
- Slate Robotics - 
  [How to implement ros_control on a custom robot](https://slaterobotics.medium.com/how-to-implement-ros-control-on-a-custom-robot-748b52751f2e)
- ICS -
  [How to Control GPIO Hardware from C or C++](https://www.ics.com/blog/how-control-gpio-hardware-c-or-c)