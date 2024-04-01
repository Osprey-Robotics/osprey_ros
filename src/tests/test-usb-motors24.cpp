#include <bitset>
#include <iomanip>
#include <iostream>
#include <unistd.h>
#include <vector>

#include "robot24.hpp"

extern unsigned char SPEED[];
extern unsigned char ACTUATE[];

/**
 * @brief Test program to run the USB motors at various speeds
 */
int main(void)
{
    int transferred;

    osprey_robotics::USB usb = osprey_robotics::USB();

    usb.loadDevices(MOTOR_USB_VID, MOTOR_USB_PID);
    usb.printDevices();

    for(int s = -50; s < 51 ; s++)
    {
        _Float32 speed = - s / 100.0f;
	    unsigned char const *cFloat = reinterpret_cast<unsigned char const *>(&speed);

        std::cout << "Trying speed " << (_Float32)speed << " hex 0x";
        for (std::size_t i = 0; i != sizeof(float); ++i)
        {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (uint32_t)cFloat[i];
        }
	    std::cout << std::endl;

        // set speed
        SPEED[12] = cFloat[0];
        SPEED[13] = cFloat[1];
        SPEED[14] = cFloat[2];
        SPEED[15] = cFloat[3];

        std::cout << "binary ";
        for (std::size_t i = 0; i != sizeof(SPEED); ++i)
        {
            std::cout << std::bitset<8>(SPEED[i]);
        }
	    std::cout << std::endl;

        // set speed
        usb.bulkWrite(SERIAL_FRONT_LEFT_1, MOTOR_USB_ENDPOINT, SPEED, sizeof(SPEED), &transferred, 1000);
        usb.bulkWrite(SERIAL_BACK_LEFT_2,  MOTOR_USB_ENDPOINT, SPEED, sizeof(SPEED), &transferred, 1000);
        usb.bulkWrite(SERIAL_FRONT_RIGHT_3, MOTOR_USB_ENDPOINT, SPEED, sizeof(SPEED), &transferred, 1000);
        usb.bulkWrite(SERIAL_BACK_RIGHT_4,  MOTOR_USB_ENDPOINT, SPEED, sizeof(SPEED), &transferred, 1000);

        // activate motor
        usb.bulkWrite(SERIAL_FRONT_LEFT_1, MOTOR_USB_ENDPOINT, ACTUATE, sizeof(ACTUATE), &transferred, 1000);
        usb.bulkWrite(SERIAL_BACK_LEFT_2,  MOTOR_USB_ENDPOINT, ACTUATE, sizeof(ACTUATE), &transferred, 1000);
        usb.bulkWrite(SERIAL_FRONT_RIGHT_3, MOTOR_USB_ENDPOINT, ACTUATE, sizeof(ACTUATE), &transferred, 1000);
        usb.bulkWrite(SERIAL_BACK_RIGHT_4,  MOTOR_USB_ENDPOINT, ACTUATE, sizeof(ACTUATE), &transferred, 1000);
        std::cout << std::endl;
        sleep(1);
    }

}
