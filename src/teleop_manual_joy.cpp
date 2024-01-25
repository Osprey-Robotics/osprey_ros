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
        publisher_cmd_vel_ = this->create_publisher<std_msgs::msg::String>("cmd_vel", 10);
        publisher_gpio_ = this->create_publisher<std_msgs::msg::String>("gpio_controller/commands", 10);
        publisher_pos_ = this->create_publisher<std_msgs::msg::String>("position_controllers/commands", 10);
        publisher_vel_ = this->create_publisher<std_msgs::msg::String>("velocity_controllers/commands", 10);

        // subscribe to /joy topic for joystick messages
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "joy", rclcpp::QoS(10), std::bind(&TeleopManualJoy::topic_callback, this, _1));

        timer_ = this->create_wall_timer(500ms,
                                         std::bind(&TeleopManualJoy::timer_callback,
                                         this));
    }

    TeleopManualJoy::~TeleopManualJoy()
    {
    }

    void TeleopManualJoy::timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Message Data" + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_cmd_vel_->publish(message);
        publisher_gpio_->publish(message);
        publisher_pos_->publish(message);
        publisher_vel_->publish(message);
    }

    void TeleopManualJoy::topic_callback(const std_msgs::msg::String & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
}
