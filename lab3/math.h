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

/*
 *  biped namespace.
 */
namespace biped
{
double
clamp(const double& data, const double& lower_bound, const double& upper_bound);
}   // namespace biped

#endif  // UTILITY_MATH_H_
