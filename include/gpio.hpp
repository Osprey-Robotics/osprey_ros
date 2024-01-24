/** Copyright 2024 Osprey Robotics - UNF
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef OSPREY_ROBOTICS__GPIO_HPP
#define OSPREY_ROBOTICS__GPIO_HPP

#include <cstddef>
#include <cstdint>

namespace osprey_robotics
{
    /**
     * @brief Class to interface with GPIO devices, sensors and relays
     */
    class GPIO
    {
        public:
            static constexpr uint8_t IN = 1;
            static constexpr uint8_t OUT = 0;
            static constexpr uint8_t HIGH = 1;
            static constexpr uint8_t LOW = 0;

            /**
             * @brief Construct a new GPIO object
             */
            GPIO();

            /**
             * @brief Construct a new GPIO object, handles initialization
             *
             * @param number GPIO device number normal range 1-26
             * @param in_out Initialize for communication in or or out
             */
            GPIO(const char *number, bool in_out);

            /**
             * @brief Destroy the GPIO object, closes device handle
             */
            virtual ~GPIO();

            /**
             * @brief Get the state of the GPIO to either high 1 or low 0
             *
             * @return true for high state
             * @return false for low state
             */
            bool input();

            /**
             * @brief Set the Direction of the GPIO communication in or out
             *
             * @param in_out true is for in, false is for out
             */
            void setDirection(bool in_out);

            /**
             * @brief Set the state of the GPIO to either high 1 or low 0
             *
             * @param high_low true is for high, false is for low
             */
            void output(bool high_low);

        private:
            int fd;
            char *direction;
            char *gpio_number;
            ssize_t gpio_size;
    };
}

#endif
