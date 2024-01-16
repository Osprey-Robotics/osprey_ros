
#include <cstdio>
#include <cstring>
#include <unistd.h>

#include "gpio.hpp"

/**
 * @brief Test program to run the USB motors at various speeds
 *
 * @param argc number of args
 * @param argv string array of args passed on command line
 *
 * @return int program return status/value
 */
int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("Usage %s <forward or backward>\n", argv[0]);
        return 1;
    }

    osprey_robotics::GPIO relay1 = osprey_robotics::GPIO(RELAY_1, osprey_robotics::GPIO::OUT);
    osprey_robotics::GPIO relay2 = osprey_robotics::GPIO(RELAY_2, osprey_robotics::GPIO::OUT);

    if (strncmp(argv[1], "forward", 7) == 0)
        relay1.output(osprey_robotics::GPIO::HIGH);
    else if (strncmp(argv[1], "backward", 8) == 0)
        relay2.output(osprey_robotics::GPIO::HIGH);

    sleep(1);

    // safety stop
    relay1.output(osprey_robotics::GPIO::LOW);
    relay2.output(osprey_robotics::GPIO::LOW);
}
