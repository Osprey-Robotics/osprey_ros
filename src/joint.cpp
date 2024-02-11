/** Copyright 2023 Osprey Robotics - UNF
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*/

#include <cstring>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <stdexcept>
#include <stdlib.h>
#include <unistd.h>

#include "gpio.hpp"
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
            if(effort == 0)
            {
                goto set_speed;
            }

            if(name == "bucket_ladder_lift_joint")
            {
                // check limit switches, limit fd I/O with extended conditional
                if(effort > 0 && gpios[0]->input() == GPIO::HIGH)
                {
                    return;
                }
                else if(effort < 0 && gpios[1]->input() == GPIO::HIGH)
                {
                    return;
                }
            }
            else if(name == "bucket_dump_joint")
            {
                // check limit switches, limit fd I/O with extended conditional
                if(effort < 0 && gpios[0]->input() == GPIO::HIGH)
                {
                    return;
                }
                else if(effort > 0 && gpios[1]->input() == GPIO::HIGH)
                {
                    return;
                }
            }

            if (effort > 50.0)
                effort = 50.0;
            if (effort < -50.0)
                effort = -50.0;

            set_speed:
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
        }
        else if (actuatorType_ == ACTUATOR_TYPE_LINEAR)
        {
            GPIO *relay = NULL;

            if (effort == 0.0)
            {
                 // stop
                gpios[0]->output(GPIO::LOW);
                gpios[1]->output(GPIO::LOW);
                return;
            }
            //extend, forward
            else if (effort == 1.0)
            {
                if(gpios[2]->input() == GPIO::HIGH)
                {
                    return;
                }
                relay = gpios[0];
            }
            // retract, backward
            else if (effort == -1.0)
            {
                relay = gpios[1];
            }

            relay->output(GPIO::HIGH);
        }
    }
}
