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
        static USB usb = USB();
        usb.loadDevices(MOTOR_USB_VID, MOTOR_USB_PID);

        static std::vector<GPIO *> gpios;
        static GPIO relay1 = GPIO(RELAY_1, GPIO::OUT);
        gpios.resize(2);
        gpios[0] = &relay1;
        static GPIO relay2 = GPIO(RELAY_2, GPIO::OUT);
        gpios[1] = &relay2;

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
        base.joints[4] = Joint(5, SERIAL_LADDER_LIFT, ACTUATOR_TYPE_MOTOR);
        base.joints[4].name = "bucket_ladder_lift_joint";
        base.joints[4].usb = &usb;
        base.joints[5] = Joint(6, SERIAL_LADDER_DIG, ACTUATOR_TYPE_MOTOR);
        base.joints[5].name = "bucket_ladder_buckets_joint";
        base.joints[5].usb = &usb;
        base.joints[6] = Joint(7, SERIAL_DEPOSITION, ACTUATOR_TYPE_MOTOR);
        base.joints[6].name = "bucket_dump_joint";
        base.joints[6].usb = &usb;
        base.joints[7] = Joint(8, "", ACTUATOR_TYPE_LINEAR);
        base.joints[7].name = "bucket_ladder_frame_joint";
        base.joints[7].gpios = gpios;
        base.joints[7].usb = NULL;
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
