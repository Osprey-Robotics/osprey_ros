/** Copyright 2023 Osprey Robotics - UNF
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*/

#include <iomanip>
#include <iostream>
#include <math.h>
#include <stdexcept>
#include <stdlib.h>

#include "joint.hpp"

// Spark Max USB speed of motors command range -0.50 to 0.50 big endian
unsigned char SPEED[] = { 0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0x05, 0x82,
                          0x08, 0x00, 0x00, 0x00,
// Begin Speed -----------------------------------------------------------------
                          0x00, 0x00, 0x00, 0x00,
// End Speed -------------------------------------------------------------------
                          0x00, 0x00, 0x00, 0x00, 0x0a, 0xf6, 0x9a, 0xfb
                        };
// Spark Max USB activate/actuate motors command
unsigned char ACTUATE[] = { 0x00, 0x00, 0x00, 0x00, 0x80, 0x2c, 0x05, 0x82,
                            0x08, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x95, 0xe9, 0x21, 0x11
                          };

namespace osprey_robotics
{

    Joint::Joint()
    {
    }

    Joint::Joint(uint8_t id, std::string serial, uint8_t actuatorType)
    {
        this->id_ = id;
        this->serial_ = serial;
        this->actuatorType_ = actuatorType;
    }

    Joint::~Joint()
    {
    }

    uint8_t Joint::getId()
    {
        return this->id_;
    }

    std::string Joint::getSerial()
    {
        return this->serial_;
    }

    void Joint::actuate(double effort, uint8_t /*duration = 1*/)
    {
        int transferred;

        if (actuatorType_ == ACTUATOR_TYPE_MOTOR)
        {
            if (effort > 50.0)
                effort = 50.0;
            if (effort < -50.0)
                effort = -50.0;

            // reduce range to -0.50 to 0.50
            _Float32 speed = (_Float32)effort / 100.0f;
            unsigned char const *cFloat = reinterpret_cast<unsigned char const *>(&speed);

            // set speed in command
            SPEED[12] = cFloat[0];
            SPEED[13] = cFloat[1];
            SPEED[14] = cFloat[2];
            SPEED[15] = cFloat[3];

            // write speed to Spark Max Controller
            usb->bulkWrite(this->serial_, MOTOR_USB_ENDPOINT, SPEED, sizeof(SPEED), &transferred, 1000);
            // activate motor
            usb->bulkWrite(this->serial_, MOTOR_USB_ENDPOINT, ACTUATE, sizeof(ACTUATE), &transferred, 1000);

            printf("transferred: [%i]; effort: [%f]; motor: %i, speed: %f",
                    transferred, effort, this->id_, (double)speed);
        }
    }
}
