
#include <iomanip>
#include <iostream>
#include <string.h>
#include <sstream>
#include <errno.h>

#include "usb.hpp"

namespace osprey_robotics
{
	USB::USB()
	{
        int rc;

        rc = libusb_init(&_usbContext);
		if (rc < 0)
		{
			std::cerr << "Failed to init USB context Error : " << rc << " - " << strerror(errno);
		}

        // libusb_set_debug(_usbContext, LIBUSB_LOG_LEVEL_DEBUG);
    }

	USB::~USB()
	{
        for (const auto& [serial, handle] : _devices)
        {
            libusb_close(handle);
        }
        libusb_exit(_usbContext);
    }

    uint8_t USB::loadDevices(uint16_t vid, uint16_t pid)
    {
        auto *data = new uint8_t[33]();
        libusb_device **devs;
        libusb_device *dev;
	    ssize_t cnt;
        int i = 0;

        cnt = libusb_get_device_list(NULL, &devs);
        if (cnt < 0){
            return 0;
        }

        while ((dev = devs[i++]) != NULL)
        {
            int r;
            struct libusb_device_descriptor desc;

            r = libusb_get_device_descriptor(dev, &desc);
            if (r < 0)
            {
                fprintf(stderr, "failed to get device descriptor %d - %s\n", errno, strerror(errno));
                return 0;
            }

            if (vid == desc.idVendor || pid == desc.idProduct)
            {
                struct libusb_device_handle *handle{ nullptr };

                r = libusb_open(dev, &handle);
                if (r < 0)
                {
                    fprintf(stderr, "failed to open device %d - %s\n", errno, strerror(errno));
                    continue;
                }

                r = libusb_reset_device(handle);
                if (r < 0)
                {
                    fprintf(stderr, "failed to reset device %d - %s\n", errno, strerror(errno));
                    continue;
                }

                r = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, data, 31);
                if (r < 0)
                {
                    fprintf(stderr, "failed to get device descriptor ascii %d - %s\n", errno, strerror(errno));
                    continue;
                }

                data[32] = '\0';
                std::stringstream serial;
                serial << data;
                _devices[serial.str()] = handle;
            }
        }
        return _devices.size();
    }

    void USB::printDevices()
    {
        struct libusb_device_descriptor desc;
        libusb_device *dev;
        int j = 0;
        int r = 0;
        uint8_t path[8];

        for (const auto& [serial, handle] : _devices)
        {
            dev = libusb_get_device(handle);

            r = libusb_get_device_descriptor(dev, &desc);
            if (r < 0)
            {
                fprintf(stderr, "failed to get device descriptor %d - %s\n", errno, strerror(errno));
                return;
            }

            printf("%04x:%04x %s (bus %d, device %d)",
                   desc.idVendor, desc.idProduct, serial.c_str(),
                   libusb_get_bus_number(dev), libusb_get_device_address(dev));

            r = libusb_get_port_numbers(dev, path, sizeof(path));
            if (r > 0)
            {
                printf(" path: %d", path[0]);
                for (j = 1; j < r; j++)
                    printf(".%d", path[j]);
            }
            printf("\n");
        }
    }

    uint8_t USB::bulkWrite(std::string serial,
                           unsigned char endpoint,
                           unsigned char* data,
                           int length,
                           int* transferred,
                           unsigned int timeout = 1000)
    {
        int r = 0;

        r = libusb_claim_interface(_devices[serial], 0);
        if (r < 0)
        {
            fprintf(stderr, "failed to claim interface %s, %d - %s\n",
                    serial.c_str(), errno, strerror(errno));
            return 0;
        }

        r = libusb_bulk_transfer(_devices[serial], endpoint, data, length, transferred, timeout);
        if(r == 0 && *transferred == length)
        {
            printf("Write successful to %s, sent %d bytes : ", serial.c_str(), *transferred);
            std::stringstream ss;
            for (int i = 0; i < length; i++)
            {
                ss << std::hex << std::setw(2) << std::setfill('0') << (uint32_t)data[i];
            }
            std::cout << ss.str() << std::endl;
        }
        else
            printf("Error in write to %s! %d : %s and transferred = %d\n",
                   serial.c_str(), errno, strerror(errno), *transferred);

        r = libusb_release_interface(_devices[serial], 0);
        if (r < 0)
        {
            fprintf(stderr, "failed to release interface %s, %d - %s\n",
                    serial.c_str(), errno, strerror(errno));
        }

        return *transferred;
    }
}
