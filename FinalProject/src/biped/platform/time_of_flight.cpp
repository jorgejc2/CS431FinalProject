/*
 * time_of_flight.cpp
 *
 *  Created on: Jan 10, 2023
 *      Author: simonyu
 */

#include "utility/math.h"
#include "platform/serial.h"
#include "platform/time_of_flight.h"

namespace biped
{
TimeOfFlight::TimeOfFlight(const uint8_t& address, const uint8_t& shutdown_pin,
        const std::shared_ptr<IOExpander> io_expander) :
        vl53l4cx_(&Wire, shutdown_pin, io_expander->getRaw())
{
    vl53l4cx_.begin();

    if (const auto error = vl53l4cx_.InitSensor(address << 1))
    {
        Serial(LogLevel::error) << "Failed to initialize time-of-flight sensor."
                << static_cast<int>(error) << " " << static_cast<int>(address);
        return;
    }

    if (vl53l4cx_.VL53L4CX_StartMeasurement())
    {
        Serial(LogLevel::error) << "Failed to start time-of-flight sensor measurement.";
        return;
    }
}

void
TimeOfFlight::read(std::vector<double>& ranges)
{
    uint8_t measurement_data_ready = 0;
    VL53L4CX_MultiRangingData_t multi_ranging_data;
    VL53L4CX_Error status = VL53L4CX_ERROR_NONE;

    ranges.clear();
    status = vl53l4cx_.VL53L4CX_GetMeasurementDataReady(&measurement_data_ready);

    if (status == VL53L4CX_ERROR_NONE && measurement_data_ready)
    {
        status = vl53l4cx_.VL53L4CX_GetMultiRangingData(&multi_ranging_data);

        if (multi_ranging_data.NumberOfObjectsFound > 0)
        {
            for (int i = 0; i < multi_ranging_data.NumberOfObjectsFound; i ++)
            {
                if (multi_ranging_data.RangeData[i].RangeStatus == 0)
                {
                    ranges.push_back(
                            millimetersToMeters(multi_ranging_data.RangeData[i].RangeMilliMeter));
                }
            }
        }

        if (status == VL53L4CX_ERROR_NONE)
        {
            status = vl53l4cx_.VL53L4CX_ClearInterruptAndStartMeasurement();
        }
    }
}
}
