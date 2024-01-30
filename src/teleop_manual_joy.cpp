/** Copyright 2024 Osprey Robotics - UNF
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include <chrono>
#include <functional>

#include "teleop_manual_joy.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace teleop_manual_joy
{
    TeleopManualJoy::TeleopManualJoy(const rclcpp::NodeOptions &options)
        : Node("teleop_manual_joy"), count_(0)
    {
        // publish to controller specific topics, may combine into unified
        publisher_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        publisher_gpio_ = this->create_publisher<std_msgs::msg::String>("gpio_controller/commands", 10);
        publisher_pos_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("position_controllers/commands", 10);
        publisher_vel_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("velocity_controllers/commands", 10);

        // subscribe to /joy topic for joystick messages
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", rclcpp::QoS(10), std::bind(&TeleopManualJoy::joyCallback, this, _1));
    }

    TeleopManualJoy::~TeleopManualJoy()
    {
    }

    void TeleopManualJoy::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        // Initializes with zeros by default.
        auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();

        cmd_vel_msg->linear.x = joy_msg->axes[TeleopManualJoy::axes::LEFT_JOY_Y];
        cmd_vel_msg->angular.z = joy_msg->axes[TeleopManualJoy::axes::LEFT_JOY_X];

        publisher_cmd_vel_->publish(std::move(cmd_vel_msg));
    }
}
