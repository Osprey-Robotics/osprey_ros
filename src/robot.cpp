/** Copyright 2023 Osprey Robotics - UNF
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*/

#include <stdexcept>

#include "robot.hpp"

namespace osprey_robotics
{
    Robot::Robot()
    {
        // shared USB interface and devices
        USB usb = USB();
        usb.loadDevices(MOTOR_USB_VID, MOTOR_USB_PID);

        // robot base
        base.joints[0] = Joint(1, SERIAL_FRONT_LEFT_1, ACTUATOR_TYPE_MOTOR);
        base.joints[0].name = "front_left_wheel_joint";
        base.joints[0].usb = &usb;
        base.joints[1] = Joint(2, SERIAL_FRONT_RIGHT_3, ACTUATOR_TYPE_MOTOR);
        base.joints[1].name = "front_right_wheel_joint";
        base.joints[1].usb = &usb;
        base.joints[2] = Joint(3, SERIAL_BACK_LEFT_2, ACTUATOR_TYPE_MOTOR);
        base.joints[2].name = "rear_left_wheel_joint";
        base.joints[2].usb = &usb;
        base.joints[3] = Joint(4, SERIAL_BACK_RIGHT_4, ACTUATOR_TYPE_MOTOR);
        base.joints[3].name = "rear_right_wheel_joint";
        base.joints[3].usb = &usb;
    }

    Robot::~Robot()
    {

    }

    Joint Robot::getJoint(std::string jointName)
    {
        int numJointsBase = sizeof(base.joints) / sizeof(base.joints[0]);
        for (int i = 0; i < numJointsBase; i++)
        {
            if (base.joints[i].name == jointName)
            {
                return base.joints[i];
            }
        }

        throw std::runtime_error("Could not find joint with name " + jointName);
    }
}
