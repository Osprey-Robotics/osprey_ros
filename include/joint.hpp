/** Copyright 2023 Osprey Robotics - UNF
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*/

#ifndef OSPREY_ROBOTICS__JOINT_HPP
#define OSPREY_ROBOTICS__JOINT_HPP

#include <cstdint>
#include <string>

#include "gpio.hpp"
#include "usb.hpp"

// types of actuators
#define ACTUATOR_TYPE_NONE -1
#define ACTUATOR_TYPE_MOTOR 0
#define ACTUATOR_TYPE_LINEAR 1

// Spark Max USB
#define MOTOR_USB_LEN 24
#define MOTOR_USB_PID 0xA30E
#define MOTOR_USB_VID 0x0483
#define MOTOR_USB_ENDPOINT 0x02

// values in joint.cpp
// speed of motors range -0.50 to 0.50 big endian
extern unsigned char SPEED[MOTOR_USB_LEN];
// activate motors
extern unsigned char ACTUATE[MOTOR_USB_LEN];

namespace osprey_robotics
{
    /**
     * @brief Class to interface with and represent motors, actuators, and other
     *        joints connected to the robot 
     */
    class Joint
    {
        public:
            std::string name;
            USB *usb;

            /**
             * @brief Construct a new Joint object, empty/unused
             */
            Joint();

            /**
             * @brief Construct a new Joint object, primary means to create a
             *        new joint with a read only id, USB serial number, and
             *        actuator type.
             * 
             * @param id internal joint id, read only after creation
             * @param serial usb serial number, read only after creation
             * @param actuatorType type of actuator from DEFINES; motor, actuator, etc
             */
            Joint(uint8_t id, std::string serial, uint8_t actuatorType);

            /**
             * @brief Destroy the Joint object, empty/unused
             */
            ~Joint();

            /**
             * @brief 
             * 
             * @param effort 
             * @param duration 
             */
            void actuate(double effort, uint8_t duration);

            /**
             * @brief Get the internal id of the joint
             * 
             * @return uint8_t the internal id number of the joint
             */
            uint8_t getId();

            /**
             * @brief Get the Serial object
             * 
             * @return std::string the USB serial number of the joint/device
             */
            std::string getSerial();

        private:
            std::string serial_;
            uint8_t id_ = 0;
            uint8_t actuatorType_ = 0;
    };
}
#endif