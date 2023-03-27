/*
 * encoder.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: simonyu
 */

#include <Arduino.h>

#include "platform/encoder.h"
#include "common/global.h"
#include "common/parameter.h"
#include "common/pin.h"
#include "platform/serial.h"

namespace biped
{
Encoder::Encoder() : steps_left_(0), steps_right_(0)
{
    /*
     *  Set pin mode for encoder and ultrasound pins.
     *  See the Pin enum for details.
     */
    // TODO LAB 6 YOUR CODE HERE.

    /*
     *  Configure X velocity low-pass filter.
     */
    low_pass_filter_velocity_x_.setBeta(EncoderParameter::low_pass_filter_beta);
}

EncoderData
Encoder::getData() const
{
    return data_;
}

void
Encoder::read()
{
    /*
     *  Take an average between the left and right total
     *  encoder step counters, convert the averaged total
     *  encoder steps into meters, and populate the
     *  corresponding entry in the member sensor data struct.
     *
     *  3700 encoder steps = 1 meter translational movement (from experiment.)
     */
    // TODO LAB 6 YOUR CODE HERE.
}

void
Encoder::calculateVelocity()
{
    static long steps_last = 0;

    read();

    /*
     *  Take an average between the left and right slow domain
     *  encoder step counters, convert the averaged slow domain
     *  encoder steps into meters, divide the converted average
     *  with the slow domain period to get the raw X velocity,
     *  filter the raw X velocity using the low-pass filter, and
     *  then finally populate the corresponding entry in the
     *  member sensor data struct.
     *
     *  3700 encoder steps = 1 meter translational movement (from experiment.)
     */
    // TODO LAB 6 YOUR CODE HERE.
}

void
Encoder::onChangeLeftA()
{
    digitalRead(ESP32Pin::motor_left_encoder_a) != digitalRead(ESP32Pin::motor_left_encoder_b) ?
            steps_left_ ++ : steps_left_ --;
}

void
Encoder::onChangeLeftB()
{
    digitalRead(ESP32Pin::motor_left_encoder_b) == digitalRead(ESP32Pin::motor_left_encoder_a) ?
            steps_left_ ++ : steps_left_ --;
}

void
Encoder::onChangeRightA()
{
    digitalRead(ESP32Pin::motor_right_encoder_a) == digitalRead(ESP32Pin::motor_right_encoder_b) ?
            steps_right_ ++ : steps_right_ --;
}

void
Encoder::onChangeRightB()
{
    digitalRead(ESP32Pin::motor_right_encoder_b) != digitalRead(ESP32Pin::motor_right_encoder_a) ?
            steps_right_ ++ : steps_right_ --;
}
}
