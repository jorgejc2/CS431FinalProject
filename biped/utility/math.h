/**
 *  @file   math.h
 *  @author Simon Yu
 *  @date   01/18/2022
 *  @brief  Math header.
 *
 *  This file defines the Math functions.
 */

/*
 *  Include guard.
 */
#ifndef UTILITY_MATH_H_
#define UTILITY_MATH_H_

#include <freertos/FreeRTOS.h>

/*
 *  biped namespace.
 */
namespace biped
{
double
clamp(const double& data, const double& lower_bound, const double& upper_bound);

/**
 *  @param  degrees Data in degrees.
 *  @return Data in radians.
 *  @brief  Convert degrees to radians.
 *
 *  This function converts data from degrees to radians.
 */
double
degreesToRadians(const double& degrees);

double
millimetersToMeters(const double& millimeters);

double
microsecondsToSeconds(const double& microseconds);

double
millisecondsToSeconds(const double& milliseconds);

TickType_t
millisecondsToTicks(const double& milliseconds);

/**
 *  @param  radians Data in radians.
 *  @return Data in degrees.
 *  @brief  Convert radians to degrees.
 *
 *  This function converts data from radians to degrees.
 */
double
radiansToDegrees(const double& radians);

double
secondsToMicroseconds(const double& seconds);

double
secondsToMilliseconds(const double& seconds);

double
ticksToMilliseconds(const TickType_t& ticks);
}   // namespace biped

#endif  // UTILITY_MATH_H_
