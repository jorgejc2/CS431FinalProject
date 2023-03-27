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
#include "math.h"

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
}   // namespace biped
