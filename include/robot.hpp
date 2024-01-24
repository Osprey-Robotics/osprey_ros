/** Copyright 2023 Osprey Robotics - UNF
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef OSPREY_ROBOTICS__ROBOT_HPP
#define OSPREY_ROBOTICS__ROBOT_HPP

#include <string>

#include "segment.hpp"

extern const char *CLASS_NAME;

// limit switch GPIO pin numbers
const char *LIMIT_BUCKET_LADDER_TOP = "9";
const char *LIMIT_BUCKET_LADDER_BOTTOM = "10";
const char *LIMIT_DEPOSITION_FORWARD = "5";
const char *LIMIT_ACTUATOR_EXTENDED = "12";
const char *LIMIT_DEPOSITION_BACK = "13";

// relay GPIO pin numbers
const char *RELAY_1 = "26";
const char *RELAY_2 = "20";

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
            Segment<8> base;

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