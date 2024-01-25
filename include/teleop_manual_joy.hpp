/** Copyright 2024 Osprey Robotics - UNF
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef TELEOP_MANUAL_JOY__TELEOP_MANUAL_JOY_HPP_
#define TELEOP_MANUAL_JOY__TELEOP_MANUAL_JOY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_options.hpp"
#include "std_msgs/msg/string.hpp"

namespace teleop_manual_joy
{
    /**
     * @brief Class for joystick/gamepad remote control, maps control actions
     *        to commands and velocities
     */
    class TeleopManualJoy : public rclcpp::Node
    {
        public:
            /**
             * @brief Construct a new Teleop Manual Joy instance
             * 
             * @param options node options passed at startup
             */
            TeleopManualJoy(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

            /**
             * @brief Destroy the Teleop Manual Joy object
             * 
             */
            virtual ~TeleopManualJoy();

        private:
            size_t count_;
            // publishers - topics we publish commands to; cmd_vel, gpio, positions, and velocities
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_cmd_vel_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_gpio_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_pos_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_vel_;

            // subscriber - the joy topic we listen to for joystick buttons
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

            // timer for publishing messages to publishers, master timer for all
            rclcpp::TimerBase::SharedPtr timer_;

            /**
             * @brief Callback function for the timer, called on the timer interval
             */
            void timer_callback();

            /**
             * @brief Callback function for subscription fired when messages
             *        on the topic are heard
             * 
             * @param msg the message data that was heard
             */
            void topic_callback(const std_msgs::msg::String & msg) const;
    };
}

#endif
