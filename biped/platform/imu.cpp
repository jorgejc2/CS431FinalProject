/*
 * imu.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: simonyu
 */

#include "platform/imu.h"
#include "utility/math.h"
#include "common/parameter.h"
#include "platform/serial.h"

namespace biped
{
IMU::IMU() : bmx160_(&Wire)
{
    initializeBMX160();
    initializeMPU6050();
}

IMUData
IMU::getDataBMX160() const
{
    return bmx160_data_;
}

IMUData
IMU::getDataMPU6050() const
{
    return mpu6050_data_;
}

void
IMU::readBMX160()
{
    /*
     *  Sensor event structs.
     *
     *  sensors_event_t struct reference:
     *  http://adafruit.github.io/Adafruit_CircuitPlayground/html/structsensors__event__t.html
     */
    // TODO LAB 5 YOUR CODE HERE.
}

void
IMU::readMPU6050()
{
    /*
     *  Sensor event structs.
     *
     *  sensors_event_t struct reference:
     *  http://adafruit.github.io/Adafruit_CircuitPlayground/html/structsensors__event__t.html
     */
    sensors_event_t acceleration;
    sensors_event_t angular_velocity;
    sensors_event_t temperature;

    /*
     *  Read from MPU6050 and populate sensor event structs.
     */
    if (!mpu6050_.getEvent(&acceleration, &angular_velocity, &temperature))
    {
        Serial(LogLevel::error) << "Failed to read from MPU6050.";
        return;
    }

    /*
     *  Using the populated sensor event structs, populate
     *  the corresponding entries in the member sensor
     *  data struct.
     *
     *  Note that the raw data read from the MPU6050 might not be in the
     *  standard body reference frame. Refer to the following materials
     *  to correctly convert the raw data into the standard body
     *  reference frame.
     *
     *  Standard body reference frame:
     *  https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-refframes
     *
     *  Rotational right-hand rule:
     *  https://en.wikipedia.org/wiki/Right-hand_rule#Rotations
     */
    // TODO LAB 5 YOUR CODE HERE.


    calculateAttitudeMPU6050();
}

void
IMU::initializeBMX160()
{
    /*
     *  Initialize BMX160 and validate the initialization.
     */
    if (!bmx160_.begin())
    {
        Serial(LogLevel::error) << "Failed to initialize BMX160.";
        return;
    }

    /*
     *  Configure BMX160.
     */
    bmx160_.setAccelRange(eAccelRange_2G);
    bmx160_.setGyroRange(eGyroRange_125DPS);

    /*
     *  Perform initial BMX160 read.
     */
    // TODO LAB 5 YOUR CODE HERE.

    /*
     *  Perform initial attitude calculation.
     *  See calculateAttitude function first for details.
     */
    // TODO LAB 5 YOUR CODE HERE.

    /*
     *  Configure Y attitude Kalman filter.
     */
    bmx160_kalman_filter_attitude_y_.setAngle(radiansToDegrees(bmx160_data_.attitude_y));
    bmx160_kalman_filter_attitude_y_.setQangle(KalmanFilterParameter::q_angle);
    bmx160_kalman_filter_attitude_y_.setQbias(KalmanFilterParameter::q_bias);
    bmx160_kalman_filter_attitude_y_.setRmeasure(KalmanFilterParameter::r_measure);
}

void
IMU::initializeMPU6050()
{
    /*
     *  Initialize MPU6050 and validate the initialization.
     */
    if (!mpu6050_.begin(AddressParameter::imu_mpu6050, &Wire, 0))
    {
        Serial(LogLevel::error) << "Failed to initialize MPU6050.";
        return;
    }

    /*
     *  Configure MPU6050.
     */
    mpu6050_.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu6050_.setGyroRange(MPU6050_RANGE_250_DEG);

    /*
     *  Perform initial MPU6050 read.
     */
    // TODO LAB 5 YOUR CODE HERE.

    /*
     *  Perform initial attitude calculation.
     *  See calculateAttitude function first for details.
     */
    // TODO LAB 5 YOUR CODE HERE.

    /*
     *  Configure Y attitude Kalman filter.
     */
    mpu6050_kalman_filter_attitude_y_.setAngle(radiansToDegrees(mpu6050_data_.attitude_y));
    mpu6050_kalman_filter_attitude_y_.setQangle(KalmanFilterParameter::q_angle);
    mpu6050_kalman_filter_attitude_y_.setQbias(KalmanFilterParameter::q_bias);
    mpu6050_kalman_filter_attitude_y_.setRmeasure(KalmanFilterParameter::r_measure);
}

void
IMU::calculateAttitudeMPU6050()
{
    /*
     *  Validate fast period.
     */
    if (PeriodParameter::fast <= 0)
    {
        Serial(LogLevel::error) << "Invalid fast period.";
        return;
    }

    /*
     *  Calculate the raw Y attitude (pitch) data using
     *  the populated linear accelerations in the member
     *  sensor data struct. Refer to the following materials
     *  to correctly convert the calculated data into the
     *  standard body reference frame.
     *
     *  Note that the attitudes (roll, pitch, and yaw) are
     *  angles between two pairs of acceleration vectors. Use
     *  atan2 function instead of atan for correct signedness.
     *
     *  Remember to perform the same calculation in the
     *  constructor for initialization but populate the
     *  member sensor data struct using the raw data (unfiltered)
     *  directly, since the Kalman filter had not been initialized
     *  at that point in the constructor.
     *
     *  Standard body reference frame:
     *  https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-refframes
     *
     *  Rotational right-hand rule:
     *  https://en.wikipedia.org/wiki/Right-hand_rule#Rotations
     */
    // TODO LAB 5 YOUR CODE HERE.

    /*
     *  Filter the raw Y attitude data using the Kalman filter.
     */
    double attitude_y_raw = 0.0;
    const double attitude_y_kalman_filter = mpu6050_kalman_filter_attitude_y_.getAngle(
            radiansToDegrees(attitude_y_raw), radiansToDegrees(mpu6050_data_.angular_velocity_y),
            PeriodParameter::fast);

    /*
     *  Convert the filtered Y attitude data back to radians
     *  using the degreesToRadians function, and populate
     *  the corresponding entry in the member sensor
     *  data struct.
     */
    // TODO LAB 5 YOUR CODE HERE.
}
}
