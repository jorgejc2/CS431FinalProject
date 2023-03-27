/*
 * time_of_flight.h
 *
 *  Created on: Jan 10, 2023
 *      Author: simonyu
 */

#ifndef PLATFORM_TIME_OF_FLIGHT_H_
#define PLATFORM_TIME_OF_FLIGHT_H_

#include <vl53l4cx_class.h>

#include "platform/io_expander.h"
#include "common/type.h"

namespace biped
{
class TimeOfFlight
{
public:

    TimeOfFlight(const uint8_t& address, const uint8_t& shutdown_pin,
            const std::shared_ptr<IOExpander> io_expander);

    void
    read(std::vector<double>& ranges);

private:

    VL53L4CX vl53l4cx_;
};
}

#endif
