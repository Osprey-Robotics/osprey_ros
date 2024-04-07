/** Copyright 2024 Osprey Robotics - UNF
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef OSPREY_ROS__REMAPPER_HPP_
#define OSPREY_ROS__REMAPPER_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_options.hpp"

namespace osprey_ros
{
    /**
     * @brief Class for merging maps slam toolbox with depth camera
     */
    class ReMapper : public rclcpp::Node
    {
        public:
            const int MAP_WIDTH = 125;  // arena ~6.8m long
            const int MAP_HEIGHT = 90;  // arena ~5m wide
            const int MAP_SIZE = ReMapper::MAP_WIDTH * ReMapper::MAP_HEIGHT;

            /**
             * @brief Construct a new ReMapper object instance
             * 
             * @param options node options passed at startup
             */
            ReMapper(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

            /**
             * @brief Destroy the ReMapper object instance
             */
            virtual ~ReMapper();

        private:
            float x;
            float y;
            float z;
            float w;
            int height = MAP_HEIGHT;
            int width = MAP_WIDTH;
            std::vector<int8_t> map;
            std::vector<int8_t> slam_map;

            // publisher - /map topic we publish merged maps to
            rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_map_;
 
            // subscription - the /slam_toolbox/map topic we listen to
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_slam_map_;

            // timer
            rclcpp::TimerBase::SharedPtr timer_map;

            /**
             * @brief Callback function for publisher fired when messages
             *        on the /map topic are published
             */
            void callbackMapPublisher();

            /**
             * @brief Callback function for subscription fired when messages
             *        on the /slam_toolbox/map topic are heard
             * 
             * @param msg the map occupancy grid message data that was heard
             */
            void slamMapCallback(const nav_msgs::msg::OccupancyGrid msg);
    };
}

#endif
