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

// GPIO sizes for hardware interface
const uint8_t LIMIT_SWITCHES = 5;
const uint8_t RELAYS = 2;
const uint8_t GPIO_IN = LIMIT_SWITCHES + RELAYS;
const uint8_t GPIO_OUT = RELAYS;

// motor USB serial numbers
const std::string SERIAL_FRONT_LEFT_1 = "206D33614D43";
const std::string SERIAL_BACK_LEFT_2 = "2061376C4243";
const std::string SERIAL_FRONT_RIGHT_3 = "206B336B4E55";
const std::string SERIAL_BACK_RIGHT_4 = "205A336B4E55";
const std::string SERIAL_LADDER_DIG = "206A33544D43";
const std::string SERIAL_LADDER_LIFT = "206C395A5543";
const std::string SERIAL_DEPOSITION = "205D39515543";

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