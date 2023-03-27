/**
 *  @file   sensor.h
 *  @author Simon Yu
 *  @date   01/12/2022
 *  @brief  Sensor class header.
 *
 *  This file defines the sensor class.
 */

/*
 *  Include guard.
 */
#ifndef SENSOR_SENSOR_H_
#define SENSOR_SENSOR_H_

/*
 *  Project headers.
 */
#include "platform/encoder.h"
#include "platform/imu.h"
#include "platform/time_of_flight.h"

/*
 *  biped namespace.
 */
namespace biped
{
/**
 *  @brief  Sensor class.
 *
 *  This class provides functions for retrieving data
 *  from various sensors, such as the motion processing
 *  unit (MPU), the motor encoders, and the ultrasonic
 *  sensor, and for populating data into the sensor data
 *  struct. The class also provides callback functions
 *  for interrupt-based sensors, such as the encoders
 *  and the ultrasonic sensor.
 */
class Sensor
{
public:

    /**
     *  @brief  Sensor class constructor.
     *
     *  This constructor initializes all class member variables.
     *  Additionally, the constructor initializes the related Arduino
     *  I/O pins as well as the MPU and the ultrasonic sensor.
     */
    Sensor();

    /**
     *  @return Encoder data struct.
     *  @brief  Get the class member encoder data strut.
     *
     *  This function returns the class member encoder data strut.
     */
    EncoderData
    getEncoderData() const;

    /**
     *  @return IMU data struct.
     *  @brief  Get the class member IMU data strut.
     *
     *  This function returns the class member IMU data strut.
     */
    IMUData
    getIMUDataBMX160() const;

    /**
     *  @return IMU data struct.
     *  @brief  Get the class member IMU data strut.
     *
     *  This function returns the class member IMU data strut.
     */
    IMUData
    getIMUDataMPU6050() const;

    /**
     *  @return Time-of-flight data struct.
     *  @brief  Get the class member time-of-flight data strut.
     *
     *  This function returns the class member time-of-flight data strut.
     */
    TimeOfFlightData
    getTimeOfFlightData() const;

    /**
     *  @param  fast_domain Whether to perform fast domain sensing.
     *  @brief  Sensor data acquisition function.
     *
     *  This function performs acquisition of all sensor data, for
     *  fast or slow domain, and populates the member sensor
     *  data struct. This function is expected to be called
     *  periodically.
     */
    void
    sense(const bool& fast_domain);

    /**
     *  @brief  Left encoder A callback function.
     *
     *  This function processes the interrupt from the left motor encoder.
     */
    void
    onEncoderChangeLeftA();

    /**
     *  @brief  Left encoder B callback function.
     *
     *  This function processes the interrupt from the left motor encoder.
     */
    void
    onEncoderChangeLeftB();

    /**
     *  @brief  Right encoder A callback function.
     *
     *  This function processes the interrupt from the right motor encoder.
     */
    void
    onEncoderChangeRightA();

    /**
     *  @brief  Right encoder B callback function.
     *
     *  This function processes the interrupt from the right motor encoder.
     */
    void
    onEncoderChangeRightB();

private:

    Encoder encoder_;
    IMU imu_;
    TimeOfFlightData time_of_flight_data_;
    std::unique_ptr<TimeOfFlight> time_of_flight_left_;
    std::unique_ptr<TimeOfFlight> time_of_flight_middle_;
    std::unique_ptr<TimeOfFlight> time_of_flight_right_;
};
}   // namespace biped

#endif  // SENSOR_SENSOR_H_
