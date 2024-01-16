
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>
#include <map>

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
        printf("Usage %s <seconds>\n", argv[0]);
        return 1;
    }

    int seconds = atoi(argv[1]);

    osprey_robotics::GPIO limit1 = osprey_robotics::GPIO(LIMIT_BUCKET_LADDER_TOP, osprey_robotics::GPIO::IN);
    osprey_robotics::GPIO limit2 = osprey_robotics::GPIO(LIMIT_BUCKET_LADDER_BOTTOM, osprey_robotics::GPIO::IN);
    osprey_robotics::GPIO limit3 = osprey_robotics::GPIO(LIMIT_DEPOSITION_FORWARD, osprey_robotics::GPIO::IN);
    osprey_robotics::GPIO limit4 = osprey_robotics::GPIO(LIMIT_ACTUATOR_EXTENDED, osprey_robotics::GPIO::IN);
    osprey_robotics::GPIO limit5 = osprey_robotics::GPIO(LIMIT_DEPOSITION_BACK, osprey_robotics::GPIO::IN);

    std::map<std::string, osprey_robotics::GPIO *> limits = {
        {"LIMIT_BUCKET_LADDER_TOP", &limit1},
        {"LIMIT_BUCKET_LADDER_BOTTOM", &limit2},
        {"LIMIT_DEPOSITION_FORWARD", &limit3},
        {"LIMIT_ACTUATOR_EXTENDED", &limit4},
        {"LIMIT_DEPOSITION_BACK", &limit5}
    };

    for (int i = 0; i < seconds; i++)
    {
        for (const auto &[name, limit] : limits)
        {
            if (limit->input())
                printf("limit switch %s is high\n", name.c_str());
            else
                printf("limit switch %s is low\n", name.c_str());
        }
        sleep(1);
    }
}
