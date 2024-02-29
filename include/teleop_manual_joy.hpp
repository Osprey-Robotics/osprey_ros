/** Copyright 2024 Osprey Robotics - UNF
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef TELEOP_MANUAL_JOY__TELEOP_MANUAL_JOY_HPP_
#define TELEOP_MANUAL_JOY__TELEOP_MANUAL_JOY_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_options.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
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
             * @brief DPad and Joystick axes on the Logitech F310 gamepad
             */
            enum axes
            {
                LEFT_JOY_X,
                LEFT_JOY_Y,
                LEFT_TRIGGER,
                RIGHT_JOY_X,
                RIGHT_JOY_Y,
                RIGHT_TRIGGER,
                DPAD_X,
                DPAD_Y
            };

            /**
             * @brief Buttons on the Logitech F310 gamepad
             */
            enum buttons
            {
                A,
                B,
                X,
                Y,
                LEFT_BUMPER,
                RIGHT_BUMPER,
                BACK,
                START,
                CENTER,
                LEFT_JOY_CLICK,
                RIGHT_JOY_CLICK
            };

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
            // publishers - topics we publish commands to; cmd_vel, gpio, positions, and velocities
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_gpio_;
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_pos_;
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_vel_;

            // subscriber - the joy topic we listen to for joystick buttons
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

            /**
             * @brief Callback function for subscription fired when messages
             *        on the joy topic are heard
             * 
             * @param msg the message data that was heard
             */
            void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    };
}

#endif
