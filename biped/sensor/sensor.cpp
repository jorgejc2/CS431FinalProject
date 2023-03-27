/**
 *  @file   sensor.cpp
 *  @author Simon Yu
 *  @date   01/12/2022
 *  @brief  Sensor class source.
 *
 *  This file implements the sensor class.
 */

/*
 *  Project headers.
 */
#include "common/global.h"
#include "common/type.h"
#include "platform/display.h"
#include "platform/io_expander.h"
#include "common/parameter.h"
#include "common/pin.h"
#include "platform/serial.h"
#include "sensor/sensor.h"

/*
 *  biped namespace.
 */
namespace biped
{
Sensor::Sensor()
{
    if (io_expander_a_)
    {
        io_expander_a_->pinModePortA(IOExpanderAPortAPin::time_of_flight_left_shutdown, OUTPUT);
        io_expander_a_->pinModePortB(IOExpanderAPortBPin::time_of_flight_middle_shutdown, OUTPUT);
        io_expander_a_->pinModePortB(IOExpanderAPortBPin::time_of_flight_right_shutdown, OUTPUT);
    }

    time_of_flight_left_ = std::unique_ptr<TimeOfFlight>(
            new TimeOfFlight(AddressParameter::time_of_flight_left,
                    IOExpanderAPortAPin::time_of_flight_left_shutdown, io_expander_a_));
    time_of_flight_middle_ = std::unique_ptr<TimeOfFlight>(
            new TimeOfFlight(AddressParameter::time_of_flight_middle,
                    IOExpanderAPortBPin::time_of_flight_middle_shutdown
                            + IOExpanderParameter::num_port_pins, io_expander_a_));
    time_of_flight_right_ = std::unique_ptr<TimeOfFlight>(
            new TimeOfFlight(AddressParameter::time_of_flight_right,
                    IOExpanderAPortBPin::time_of_flight_right_shutdown
                            + IOExpanderParameter::num_port_pins, io_expander_a_));
}

EncoderData
Sensor::getEncoderData() const
{
    return encoder_.getData();
}

IMUData
Sensor::getIMUDataBMX160() const
{
    return imu_.getDataBMX160();
}

IMUData
Sensor::getIMUDataMPU6050() const
{
    return imu_.getDataMPU6050();
}

TimeOfFlightData
Sensor::getTimeOfFlightData() const
{
    return time_of_flight_data_;
}

void
Sensor::sense(const bool& fast_domain)
{
    /*
     *  The fast domain tasks are the ones that requires
     *  faster execution frequency, such as the reading of
     *  the MPU, the attitude calculation, and etc. If such
     *  tasks were performed too seldomly, it might causes
     *  inaccurate or delayed data sampling. For example, if
     *  the attitude was calculated too seldomly, then it
     *  is possible that the plant has already moved to another
     *  pose during the gap between the calculations, which could
     *  cause, for instance, the robot failing to perfectly
     *  balance itself due to the delayed sample.
     *
     *  The slow domain tasks, on the other hand, are the ones
     *  that are better executed at lower frequency due to noise or
     *  performance issues. For example, the velocity here is
     *  calculated by dividing the encoder step count per period by the
     *  period. The smaller the period, the noisier the velocity.
     *  Additionally, it is unnecessary to read less important sensors
     *  such as the ultrasonic sensor at higher frequency. Thus, it's
     *  more performance-friendly to process such tasks in slow domain.
     */
    if (fast_domain)
    {
        /*
         *  Read encoders.
         */
        // TODO LAB 5 YOUR CODE HERE.

        /*
         *  Read BMX160.
         */
        // TODO LAB 5 YOUR CODE HERE.

        /*
         *  Read MPU6050.
         */
        // TODO LAB 5 YOUR CODE HERE.
    }
    else
    {
        /*
         *  Calculate velocity.
         */
        // TODO LAB 5 YOUR CODE HERE.

        /*
         *  Read time-of-flight sensors.
         */
        // TODO LAB 5 YOUR CODE HERE.
        time_of_flight_left_->read(time_of_flight_data_.ranges_left);
        time_of_flight_middle_->read(time_of_flight_data_.ranges_middle);
        time_of_flight_right_->read(time_of_flight_data_.ranges_right);
    }
}

void
Sensor::onEncoderChangeLeftA()
{
    encoder_.onChangeLeftA();
}

void
Sensor::onEncoderChangeLeftB()
{
    encoder_.onChangeLeftB();
}

void
Sensor::onEncoderChangeRightA()
{
    encoder_.onChangeRightA();
}

void
Sensor::onEncoderChangeRightB()
{
    encoder_.onChangeRightB();
}
}   // namespace biped
