/** Copyright 2024 Osprey Robotics - UNF
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef OSPREY_ROBOTICS__ROBOT_HPP
#define OSPREY_ROBOTICS__ROBOT_HPP

#include <string>

#include "segment.hpp"

extern const char *CLASS_NAME;

// motor USB serial numbers
const std::string SERIAL_FRONT_LEFT_1 = "206B376F5557";
const std::string SERIAL_BACK_LEFT_2 = "2052376E5058";
const std::string SERIAL_FRONT_RIGHT_3 = "206F37635557";
const std::string SERIAL_BACK_RIGHT_4 = "205C37535058";
// const std::string SERIAL_DEPOSITION = "";

// motor speed multipliers min/max -50.0/50.0
const _Float32 SPEED_MULTIPLIER_DEPOSITION = 30.0;
const _Float32 SPEED_MULTIPLIER_WHEELS = 7.162;

namespace osprey_robotics
{
    /**
     * @brief Class that represents the total robot, all joints, sensors, etc.
     */
    class Robot
    {
        private:

        public:
            /**
             * @brief Construct a new Robot object, creates a new usb context
             *        and interface, then loads all USB devices, and initializes
             *        all joints, sensors, etc
             */
            Robot();

            /**
             * @brief Destroy the Robot object, empty/unused
             */
            ~Robot();

            /**
             * @brief Represents the robot
             */
            Segment<5> base;

            /**
             * @brief Get a Joint object by name
             *
             * @param jointName name of the joint
             * @return Joint
             */
            Joint getJoint(std::string jointName);
    };
}

#endif