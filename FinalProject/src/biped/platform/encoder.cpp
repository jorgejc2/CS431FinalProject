/**
 *  @file   encoder.cpp
 *  @author Simon Yu
 *  @date   01/05/2023
 *  @brief  Encoder class source.
 *
 *  This file implements the encoder class.
 */

/*
 *  External headers.
 */
#include <Arduino.h>

/*
 *  Project headers.
 */
#include "platform/encoder.h"
#include "common/global.h"
#include "task/interrupt.h"
#include "common/parameter.h"
#include "common/pin.h"
#include "platform/serial.h"

/*
 *  Biped namespace.
 */
namespace biped
{
Encoder::Encoder() : steps_left_(0), steps_right_(0)
{
    /*
     *  Set pin mode for the motor encoder pins using
     *  the Arduino pin mode function. Use pull-up if the pin
     *  mode is input.
     *  See the parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    pinMode(ESP32Pin::motor_left_encoder_a, INPUT_PULLUP);
    pinMode(ESP32Pin::motor_left_encoder_b, INPUT_PULLUP);
    pinMode(ESP32Pin::motor_right_encoder_a, INPUT_PULLUP);
    pinMode(ESP32Pin::motor_right_encoder_b, INPUT_PULLUP);
    /*
     *  Configure X velocity low-pass filter.
     */
    low_pass_filter_velocity_x_.setBeta(EncoderParameter::low_pass_filter_beta);
}

EncoderData
Encoder::getData() const
{
    /*
     *  Return the class member encoder data struct.
     */
    // TODO LAB 6 YOUR CODE HERE.
    return data_;
}

void
Encoder::read()
{
    /*
     *  Take an average between the left and right total
     *  encoder step counters to be the overall encoder
     *  step count, convert the averaged overall encoder
     *  steps into meters, and populate the corresponding
     *  entries in the member encoder data struct.
     *
     *  See the parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    data_.steps = (steps_left_ + steps_right_) / 2;
    data_.position_x = data_.steps / EncoderParameter::steps_per_meter;
}

void
Encoder::calculateVelocity()
{
    /*
     *  Declare last overall encoder step counter and initialize to 0.
     */
    static long steps_last = 0;

    /*
     *  Read encoders.
     */
    // TODO LAB 6 YOUR CODE HERE.
    read();

    /*
     *  Calculate the step count since the last overall encoder step
     *  counter update, convert the calculated steps into meters,
     *  calculate the X velocity using the slow domain period, filter
     *  the calculated X velocity using the low-pass filter, and
     *  populate the corresponding entry in the member encoder data struct.
     *
     *  See the parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    data_.velocity_x = ((data_.steps - steps_last) / PeriodParameter::slow) / EncoderParameter::steps_per_meter;
    

    /*
     *  Set last overall encoder step counter to be the current
     *  overall encoder step count.
     */
    // TODO LAB 6 YOUR CODE HERE.
    steps_last = data_.steps;
}

void IRAM_ATTR
Encoder::onLeftA()
{
    /*
     *  Read left encoder pin states using digitalReadFromISR
     *  function and increment/decrement the left encoder step
     *  counters based on the pin states read.
     *
     *  Incremental rotary encoder references:
     *  https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/
     */
    // TODO LAB 6 YOUR CODE HERE.
    int A = digitalReadFromISR(ESP32Pin::motor_left_encoder_a);
    int B = digitalReadFromISR(ESP32Pin::motor_left_encoder_b);
    if(A != B)
        steps_left_++;        
    else
        steps_left_--;
}

void IRAM_ATTR
Encoder::onLeftB()
{
    /*
     *  Read left encoder pin states using digitalReadFromISR
     *  function and increment/decrement the left encoder step
     *  counters based on the pin states read.
     *
     *  Incremental rotary encoder references:
     *  https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/
     */
    // TODO LAB 6 YOUR CODE HERE.
    int A = digitalReadFromISR(ESP32Pin::motor_left_encoder_a);
    int B = digitalReadFromISR(ESP32Pin::motor_left_encoder_b);
    if(A == B)
        steps_left_++;        
    else
        steps_left_--;
}
void IRAM_ATTR
Encoder::onRightA()
{
    /*
     *  Read right encoder pin states using digitalReadFromISR
     *  function and increment/decrement the right encoder step
     *  counters based on the pin states read.
     *
     *  Incremental rotary encoder references:
     *  https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/
     */
    // TODO LAB 6 YOUR CODE HERE.
    int A = digitalReadFromISR(ESP32Pin::motor_right_encoder_a);
    int B = digitalReadFromISR(ESP32Pin::motor_right_encoder_b);
    if(A == B)
        steps_right_++;        
    else
        steps_right_--;
}

void IRAM_ATTR
Encoder::onRightB()
{
    /*
     *  Read right encoder pin states using digitalReadFromISR
     *  function and increment/decrement the right encoder step
     *  counters based on the pin states read.
     *
     *  Incremental rotary encoder references:
     *  https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/
     */
    // TODO LAB 6 YOUR CODE HERE.
    int A = digitalReadFromISR(ESP32Pin::motor_right_encoder_a);
    int B = digitalReadFromISR(ESP32Pin::motor_right_encoder_b);
    if(A != B)
        steps_right_++;        
    else
        steps_right_--;
}
}   // namespace biped
