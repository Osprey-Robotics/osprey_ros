/** Copyright 2023 Osprey Robotics - UNF
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef OSPREY_ROBOTICS__ROBOT_HARDWARE_INTERFACE_HPP_
#define OSPREY_ROBOTICS__ROBOT_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "robot.hpp"

const char *CLASS_NAME = "RobotSystemHardware";

namespace robot_hardware_interface
{
    class RobotSystemHardware : public hardware_interface::SystemInterface
    {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(RobotSystemHardware)

            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo &info) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State &previous_state) override;

            hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State &previous_state) override;

            hardware_interface::return_type read(
                const rclcpp::Time &time, const rclcpp::Duration &period) override;

            hardware_interface::return_type write(
                const rclcpp::Time &time, const rclcpp::Duration &period) override;

        private:
            // params
            double hw_start_sec_;
            double hw_stop_sec_;

            osprey_robotics::Robot robot_;
            std::vector<double> hw_effort_;
            std::vector<double> hw_commands_;
            std::vector<double> hw_positions_;
            std::vector<double> hw_velocities_;
    };

}

#endif
