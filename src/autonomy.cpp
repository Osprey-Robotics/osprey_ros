/** Copyright 2024 Osprey Robotics - UNF
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include "autonomy.hpp"

using std::placeholders::_1;

namespace osprey_ros
{
    Autonomy::Autonomy(const rclcpp::NodeOptions &options)
        : Node("autonomy", options)
    {
        // publish to autonomy topic
        publisher_goal_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        // subscribe to /autonomy topic
        subscription_autonomy_ =
            this->create_subscription<std_msgs::msg::String>("autonomy",
                rclcpp::QoS(10), std::bind(&Autonomy::autonomyCallback, this, _1));
    }

    Autonomy::~Autonomy()
    {
    }

    void Autonomy::autonomyCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "go")
        {
            // Initializes with zeros by default.
            auto goal_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();

            goal_msg->header.stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
            goal_msg->header.frame_id = "map";
            goal_msg->pose.position.x = 3.0;
            goal_msg->pose.position.y = 3.0;
            goal_msg->pose.position.z = 0.0;
            goal_msg->pose.orientation.w = 1.0;

            // publish goal
            publisher_goal_->publish(std::move(goal_msg));
        }

    }
}
