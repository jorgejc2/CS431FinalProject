/*
 * encoder.h
 *
 *  Created on: Jan 5, 2023
 *      Author: simonyu
 */

#ifndef PLATFORM_ENCODER_H_
#define PLATFORM_ENCODER_H_

#include "utility/low_pass_filter.hpp"
#include "common/type.h"

namespace biped
{
class Encoder
{
public:

    Encoder();

    /**
     *  @return Encoder data struct.
     *  @brief  Get the class member encoder data strut.
     *
     *  This function returns the class member encoder data strut.
     */
    EncoderData
    getData() const;

    /**
     *  @brief  Encoder reading function.
     *
     *  This function reads data from the motor
     *  encoders and populates the corresponding entries
     *  in the member encoder data struct.
     */
    void
    read();

    /**
     *  @brief  Velocity calculation function.
     *
     *  This function calculates linear velocity data from encoder
     *  data and populates the corresponding entries in the member
     *  sensor data struct. The function also filters certain data
     *  using the low pass filter.
     */
    void
    calculateVelocity();

    /**
     *  @brief  Left encoder A callback function.
     *
     *  This function processes the interrupt from the left motor encoder.
     */
    void
    onChangeLeftA();

    /**
     *  @brief  Left encoder B callback function.
     *
     *  This function processes the interrupt from the left motor encoder.
     */
    void
    onChangeLeftB();

    /**
     *  @brief  Right encoder A callback function.
     *
     *  This function processes the interrupt from the right motor encoder.
     */
    void
    onChangeRightA();

    /**
     *  @brief  Right encoder B callback function.
     *
     *  This function processes the interrupt from the right motor encoder.
     */
    void
    onChangeRightB();

private:

    EncoderData data_;
    LowPassFilter<double> low_pass_filter_velocity_x_;  //!< X velocity low-pass filter object.
    long steps_left_;   //!< Left encoder step counter.
    long steps_right_;  //!< Right encoder step counter.
};
}

#endif
