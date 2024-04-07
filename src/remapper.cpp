/** Copyright 2024 Osprey Robotics - UNF
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include "remapper.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace osprey_ros
{
    ReMapper::ReMapper(const rclcpp::NodeOptions &options)
        : Node("remapper", options)
    {
        map.resize(ReMapper::MAP_SIZE);
        for( int i = 0 ; i < MAP_SIZE; i++ )
            map[i] = 0;

        // publish to /map topic
        publisher_map_ =
            this->create_publisher<nav_msgs::msg::OccupancyGrid>("map",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        timer_map = this->create_wall_timer(500ms, std::bind(&ReMapper::callbackMapPublisher, this));

        // subscribe to /slam_toolbox/map topic
        subscription_slam_map_ =
            this->create_subscription<nav_msgs::msg::OccupancyGrid>("slam_toolbox/map",
                rclcpp::QoS(10), std::bind(&ReMapper::slamMapCallback, this, _1));
    }

    ReMapper::~ReMapper()
    {
    }

    void ReMapper::callbackMapPublisher()
    {
        // Initializes with zeros by default.
        auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();

        msg->header.stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        msg->header.frame_id = "map";
        msg->info.map_load_time = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        msg->info.resolution = 0.05;
        msg->info.width = width;
        msg->info.height = height;
        msg->info.origin.position.x = x;
        msg->info.origin.position.y = y;
        msg->info.origin.position.z = z;
        msg->info.origin.orientation.w = w;
        msg->data = slam_map;

        publisher_map_->publish(std::move(msg));
    }

    void ReMapper::slamMapCallback(const nav_msgs::msg::OccupancyGrid msg)
    {
        width = msg.info.width;
        height = msg.info.height;
        x = msg.info.origin.position.x;
        y = msg.info.origin.position.y;
        z = msg.info.origin.position.z;
        w = msg.info.origin.orientation.w;
        slam_map = msg.data;
    }
}
