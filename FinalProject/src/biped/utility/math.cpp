/**
 *  @file   math.cpp
 *  @author Simon Yu
 *  @date   01/18/2022
 *  @brief  Math source.
 *
 *  This file implements the Math functions.
 */

/*
 *  External headers.
 */
#include <cmath>

/*
 *  Project headers.
 */
#include "utility/math.h"

/*
 *  biped namespace.
 */
namespace biped
{
double
clamp(const double& data, const double& lower_bound, const double& upper_bound)
{
    return data < lower_bound ? lower_bound : (data > upper_bound ? upper_bound : data);
}

double
degreesToRadians(const double& degrees)
{
    /*
     *  Convert degrees to radians.
     */
    return degrees / 180 * M_PI;
}

double
millimetersToMeters(const double& millimeters)
{
    return millimeters / 1000;
}

double
microsecondsToSeconds(const double& microseconds)
{
    return microseconds / 1000 / 1000;
}

double
millisecondsToSeconds(const double& milliseconds)
{
    return milliseconds / 1000;
}

TickType_t
millisecondsToTicks(const double& milliseconds)
{
    return pdMS_TO_TICKS(milliseconds);
}

double
radiansToDegrees(const double& radians)
{
    /*
     *  Convert radians to degrees.
     */
    return radians / M_PI * 180;
}

double
secondsToMicroseconds(const double& seconds)
{
    return secondsToMilliseconds(seconds) * 1000;
}

double
secondsToMilliseconds(const double& seconds)
{
    return seconds * 1000;
}

double
ticksToMilliseconds(const TickType_t& ticks)
{
    return ticks * portTICK_PERIOD_MS;
}
}   // namespace biped
