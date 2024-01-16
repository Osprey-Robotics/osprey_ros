
/** Copyright 2024 Osprey Robotics - UNF
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include "gpio.hpp"

namespace osprey_robotics
{
    GPIO::GPIO()
    {
    }

    GPIO::GPIO(const char *number, bool in_out)
    {
        const char *gpio_sysfsdev = "/sys/class/gpio/export";

        gpio_number = (char *)number;
        gpio_size = strlen(number);

        fd = open(gpio_sysfsdev, O_WRONLY);
        if (fd == -1)
        {
            fprintf(stderr, "Unable to open %s : %s \n", gpio_sysfsdev, strerror(errno));
            return;
        }

        if (write(fd, gpio_number, gpio_size) != gpio_size)
        {
            fprintf(stderr, "Error writing to %s gpio %s : %s \n", gpio_sysfsdev, gpio_number, strerror(errno));
            return;
        }

        close(fd);

        setDirection(in_out);
    }

    GPIO::~GPIO()
    {
        const char *gpio_sysfsdev = "/sys/class/gpio/unexport";

        fd = open(gpio_sysfsdev, O_WRONLY);
        if (fd == -1)
        {
            fprintf(stderr, "Unable to open %s : %s \n", gpio_sysfsdev, strerror(errno));
            return;
        }

        if (write(fd, gpio_number, gpio_size) != gpio_size)
        {
            fprintf(stderr, "Error writing to %s gpio %s : %s \n", gpio_sysfsdev, gpio_number, strerror(errno));
            return;
        }

        close(fd);
    }

    void GPIO::setDirection(bool in_out)
    {
        const char *SOUT = "out";
        const char *SIN = "in";
        char gpio_sysfsdev[33];
        ssize_t size;

        snprintf(gpio_sysfsdev, sizeof(gpio_sysfsdev), "/sys/class/gpio/gpio%s/direction", gpio_number);

        if (in_out)
            direction = (char *)SIN;
        else
            direction = (char *)SOUT;

        size = strlen(direction);

        fd = open(gpio_sysfsdev, O_WRONLY);
        if (fd == -1)
        {
            fprintf(stderr, "Unable to open %s : %s \n", gpio_sysfsdev, strerror(errno));
            return;
        }

        if (write(fd, direction, size) != size)
        {
            fprintf(stderr, "Error writing %s to %s : %s \n", direction, gpio_sysfsdev, strerror(errno));
            return;
        }

        close(fd);
    }

    bool GPIO::input()
    {
        char gpio_sysfsdev[29];
        char value;

        snprintf(gpio_sysfsdev, sizeof(gpio_sysfsdev), "/sys/class/gpio/gpio%s/value", gpio_number);

        fd = open(gpio_sysfsdev, O_RDONLY);
        if (fd == -1)
        {
            fprintf(stderr, "Unable to open %s : %s \n", gpio_sysfsdev, strerror(errno));
            return 0;
        }

        if (read(fd, &value, 1) != 1)
        {
            fprintf(stderr, "Error reading from %s : %s \n", gpio_sysfsdev, strerror(errno));
            return 0;
        }

        close(fd);

        if (value == '1')
            return 1;
        else
            return 0;
    }

    void GPIO::output(bool high_low)
    {
        const char S1 = '1';
        const char S0 = '0';
        char gpio_sysfsdev[29];
        char *value;

        snprintf(gpio_sysfsdev, sizeof(gpio_sysfsdev), "/sys/class/gpio/gpio%s/value", gpio_number);

        if (high_low)
            value = (char *)&S1;
        else
            value = (char *)&S0;

        fd = open(gpio_sysfsdev, O_WRONLY);
        if (fd == -1)
        {
            fprintf(stderr, "Unable to open %s : %s \n", gpio_sysfsdev, strerror(errno));
            return;
        }

        if (write(fd, value, 1) != 1)
        {
            fprintf(stderr, "Error writing to %s : %s \n", gpio_sysfsdev, strerror(errno));
            return;
        }

        close(fd);
    }

}
