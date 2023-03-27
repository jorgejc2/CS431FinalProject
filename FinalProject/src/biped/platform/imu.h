/*
 * imu.h
 *
 *  Created on: Jan 5, 2023
 *      Author: simonyu
 */

#ifndef PLATFORM_IMU_H_
#define PLATFORM_IMU_H_

#include <Adafruit_MPU6050.h>
#include <DFRobot_BMX160.h>
#include <Kalman.h>

#include "common/type.h"

namespace biped
{
class IMU
{
public:

    IMU();

    /**
     *  @return IMU data struct.
     *  @brief  Get the class member IMU data strut.
     *
     *  This function returns the class member IMU data strut.
     */
    IMUData
    getDataBMX160() const;

    /**
     *  @return IMU data struct.
     *  @brief  Get the class member IMU data strut.
     *
     *  This function returns the class member IMU data strut.
     */
    IMUData
    getDataMPU6050() const;

    /**
     *  @brief  BMX160 reading function.
     *
     *  This function reads data from the BMX160 IMU and
     *  populates the corresponding entries in the member
     *  IMU data struct.
     */
    void
    readBMX160();

    /**
     *  @brief  MPU6050 reading function.
     *
     *  This function reads data from the MPU6050 IMU and
     *  populates the corresponding entries in the member
     *  IMU data struct.
     */
    void
    readMPU6050();

private:

    void
    initializeBMX160();

    void
    initializeMPU6050();

    /**
     *  @brief  Attitude calculation function.
     *
     *  This function calculates attitude data from acceleration
     *  data and populates the corresponding entries in the member
     *  sensor data struct. The function also filters certain data
     *  using the Kalman filter.
     */
    void
    calculateAttitudeBMX160();

    /**
     *  @brief  Attitude calculation function.
     *
     *  This function calculates attitude data from acceleration
     *  data and populates the corresponding entries in the member
     *  sensor data struct. The function also filters certain data
     *  using the Kalman filter.
     */
    void
    calculateAttitudeMPU6050();

    DFRobot_BMX160 bmx160_; //!< DFRobot BMX160 object.
    IMUData bmx160_data_;    //!< BMX160 IMU data struct.
    Kalman bmx160_kalman_filter_attitude_x_;   //!< BMX160 X attitude (roll) Kalman filter object.
    Kalman bmx160_kalman_filter_attitude_y_;  //!< BMX160 Y attitude (pitch) Kalman filter object.
    Kalman bmx160_kalman_filter_attitude_z_;   //!< BMX160 Z attitude (yaw) Kalman filter object.

    Adafruit_MPU6050 mpu6050_;  //!< Adafruit MPU6050 object.
    IMUData mpu6050_data_;    //!< MPU6050 IMU data struct.
    Kalman mpu6050_kalman_filter_attitude_x_;   //!< MPU6050 X attitude (roll) Kalman filter object.
    Kalman mpu6050_kalman_filter_attitude_y_;  //!< MPU6050 Y attitude (pitch) Kalman filter object.
    Kalman mpu6050_kalman_filter_attitude_z_;   //!< MPU6050 Z attitude (yaw) Kalman filter object.
};
}

#endif
