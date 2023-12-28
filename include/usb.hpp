/** Copyright 2023 Osprey Robotics - UNF
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*/

#ifndef OSPREY_ROBOTICS__USB_HPP
#define OSPREY_ROBOTICS__USB_HPP

#include <libusb-1.0/libusb.h>
#include <string>
#include <unordered_map>

namespace osprey_robotics
{
    /**
     * @brief Class to interface with USB devices, primarily USB motors
     */
    class USB
    {
        public:
            /**
             * @brief Construct a new USB object, handles context initialization
             */
            USB();

            /**
             * @brief Destroy the USB object, closes any loaded device handles
             *        in addition to USB context exit
             */
            virtual ~USB();

            /**
             * @brief Loads USB Device handles by vendor and product IDs
             * 
             * @param vid USB Vendor ID
             * @param pid USB Product ID
             * @return uint8_t the number of devices loaded
             */
            uint8_t loadDevices(uint16_t vid, uint16_t pid);

            /**
             * @brief Print information on loaded devices
             */
            void printDevices();

            /**
             * @brief Perform a bulk transfer/write to loaded USB device
             * 
             * @param serial USB device serial number
             * @param endpoint USB device endpoint
             * @param data data to be transferred/written
             * @param length length/size of the data
             * @param transferred number of bytes transferred
             * @param timeout timeout (in milliseconds) this function should wait
             * @return uint8_t number of bytes transferred
             */
            uint8_t bulkWrite(std::string serial,
                              unsigned char endpoint,
                              unsigned char* data,
                              int length,
                              int* transferred,
                              unsigned int timeout);

        private:
            std::unordered_map<std::string, libusb_device_handle*> _devices;
            libusb_context *_usbContext;

    };
}

#endif