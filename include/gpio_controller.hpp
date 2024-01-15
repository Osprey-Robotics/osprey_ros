/** Copyright 2024 Osprey Robotics - UNF
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef OSPREY_ROBOTICS__GPIO_CONTROLLER_HPP_
#define OSPREY_ROBOTICS__GPIO_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/interface_value.hpp"
#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace gpio_controller
{
    using CmdType = std_msgs::msg::Float64MultiArray;

    class GPIOController : public controller_interface::ControllerInterface
    {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(GPIOController)

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::return_type update(const rclcpp::Time &time,
                                                     const rclcpp::Duration &period) override;

            CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

            CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

            CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

            CallbackReturn on_init() override;

        private:
            std::vector<std::string> inputs_;
            std::vector<std::string> outputs_;

        protected:
            void initMsgs();

            // internal commands
            std::shared_ptr<CmdType> output_cmd_ptr_;

            // publisher
            std::shared_ptr<rclcpp::Publisher<control_msgs::msg::InterfaceValue>> gpio_publisher_;
            control_msgs::msg::InterfaceValue gpio_msg_;

            // subscriber
            rclcpp::Subscription<CmdType>::SharedPtr subscription_command_;
    };
}

#endif
