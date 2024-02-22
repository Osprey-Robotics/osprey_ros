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

        static std::vector<GPIO *> gpios_lift;
        gpios_lift.resize(2);
        static GPIO limit_top = GPIO(LIMIT_BUCKET_LADDER_TOP, GPIO::IN);
        gpios_lift[0] = &limit_top;
        static GPIO limit_bottom = GPIO(LIMIT_BUCKET_LADDER_BOTTOM, GPIO::IN);
        gpios_lift[1] = &limit_bottom;

        static std::vector<GPIO *> gpios_dump;
        gpios_dump.resize(2);
        static GPIO limit_forward = GPIO(LIMIT_DEPOSITION_FORWARD, GPIO::IN);
        gpios_dump[0] = &limit_forward;
        static GPIO limit_back = GPIO(LIMIT_DEPOSITION_BACK, GPIO::IN);
        gpios_dump[1] = &limit_back;

        static std::vector<GPIO *> gpios_actuators;
        gpios_actuators.resize(3);
        static GPIO relay1 = GPIO(RELAY_1, GPIO::OUT);
        gpios_actuators[0] = &relay1;
        static GPIO relay2 = GPIO(RELAY_2, GPIO::OUT);
        gpios_actuators[1] = &relay2;
        static GPIO limit_extended = GPIO(LIMIT_ACTUATOR_EXTENDED, GPIO::IN);
        gpios_actuators[2] = &limit_extended;

        // robot base
        base.joints[0] = Joint(1, SERIAL_FRONT_LEFT_1, ACTUATOR_TYPE_MOTOR);
        base.joints[0].multiplier = SPEED_MULTIPLIER_WHEELS;
        base.joints[0].name = "front_left_wheel_joint";
        base.joints[0].usb = &usb;
        base.joints[1] = Joint(2, SERIAL_FRONT_RIGHT_3, ACTUATOR_TYPE_MOTOR);
        base.joints[1].multiplier = SPEED_MULTIPLIER_WHEELS;
        base.joints[1].name = "front_right_wheel_joint";
        base.joints[1].usb = &usb;
        base.joints[2] = Joint(3, SERIAL_BACK_LEFT_2, ACTUATOR_TYPE_MOTOR);
        base.joints[2].multiplier = SPEED_MULTIPLIER_WHEELS;
        base.joints[2].name = "rear_left_wheel_joint";
        base.joints[2].usb = &usb;
        base.joints[3] = Joint(4, SERIAL_BACK_RIGHT_4, ACTUATOR_TYPE_MOTOR);
        base.joints[3].multiplier = SPEED_MULTIPLIER_WHEELS;
        base.joints[3].name = "rear_right_wheel_joint";
        base.joints[3].usb = &usb;
        base.joints[4] = Joint(5, SERIAL_LADDER_LIFT, ACTUATOR_TYPE_MOTOR);
        base.joints[4].multiplier = SPEED_MULTIPLIER_LADDER_LIFT;
        base.joints[4].name = "bucket_ladder_lift_joint";
        base.joints[4].gpios = gpios_lift;
        base.joints[4].usb = &usb;
        base.joints[5] = Joint(6, SERIAL_LADDER_DIG, ACTUATOR_TYPE_MOTOR);
        base.joints[5].multiplier = SPEED_MULTIPLIER_LADDER_BUCKETS;
        base.joints[5].name = "bucket_ladder_buckets_joint";
        base.joints[5].usb = &usb;
        base.joints[6] = Joint(7, SERIAL_DEPOSITION, ACTUATOR_TYPE_MOTOR);
        base.joints[6].multiplier = SPEED_MULTIPLIER_DEPOSITION;
        base.joints[6].name = "bucket_dump_joint";
        base.joints[6].gpios = gpios_dump;
        base.joints[6].usb = &usb;
        base.joints[7] = Joint(8, "", ACTUATOR_TYPE_LINEAR);
        base.joints[7].name = "bucket_ladder_frame_joint";
        base.joints[7].gpios = gpios_actuators;
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
