/** Copyright 2023 Osprey Robotics - UNF
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*/

#ifndef OSPREY_ROBOTICS__SEGMENT_HPP
#define OSPREY_ROBOTICS__SEGMENT_HPP

#include "joint.hpp"

namespace osprey_robotics
{
    /**
     * @brief Template class that represents segments attached to the robot base
     * 
     * @tparam T the number of segments attached
     */
    template <int T> class Segment
    {
        public:
            Joint joints[T];

            /**
             * @brief Construct a new Segment object, empty/unused
             */
            Segment() { };

            /**
             * @brief Destroy the Segment object, empty/unused
             */

            ~Segment() { };

            /**
             * @brief The size of the 
             * 
             * @return int 
             */
            int size() const { return T; }

        private:
    };
}

#endif