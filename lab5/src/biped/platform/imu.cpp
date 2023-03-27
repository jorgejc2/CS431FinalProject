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

std::vector<std::vector<double>> gemm (std::vector<std::vector<double>> &m1, std::vector<std::vector<double>> &m2) {
    // m1 = m * k
    // m2 = k*n
    // output = m*n
    int m = m1.size();
    int k = m1[0].size();
    int n = m2.size();
    std::vector<std::vector<double>> output(m, std::vector<double>(n, 0));

    /* iterate through rows of output */
    for (int row = 0; row < m; row++) {
        /* iteratre through cols of output*/
        for (int col = 0; col < n; col++) {
            /* for each output */
            for (int i = 0; i < k; i++) {
                output[row][col] += m1[row][i] * m2[i][col];
            }
        }
    }

    return output;
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
    sBmx160SensorData_t magnetic;  // compass data in micro Tesla
    sBmx160SensorData_t gyro;  // roll, pitch, and yaw rates in rad/s
    sBmx160SensorData_t acceleration;  // acceleration values in meters per second squared

    bmx160_.getAllData(&magnetic, &gyro, &acceleration);

    /* update private data struct */
    // bmx160_data_.acceleration_x = acceleration.x;
    // bmx160_data_.acceleration_y = acceleration.y;
    // bmx160_data_.acceleration_z = acceleration.z;
    // bmx160_data_.angular_velocity_x = gyro.x;
    // bmx160_data_.angular_velocity_y = gyro.y;
    // bmx160_data_.angular_velocity_z = gyro.z;
    // bmx160_data_.compass_x = magnetic.x;
    // bmx160_data_.compass_y = magnetic.y;
    // bmx160_data_.compass_z = magnetic.z;
    std::vector<std::vector<double>> original_acceleration {
        {acceleration.x, acceleration.y, acceleration.z}
        };
    std::vector<std::vector<double>> original_av {
        {gyro.x, gyro.y, gyro.z}
    };
    std::vector<std::vector<double>> original_mag {
        {magnetic.x, magnetic.y, magnetic.z}
    };
    std::vector<std::vector<double>> transform {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    std::vector<std::vector<double>> accelerations = gemm(original_acceleration, transform);
    std::vector<std::vector<double>> avs = gemm(original_av, transform);
    std::vector<std::vector<double>> mags = gemm(original_mag, transform);
    bmx160_data_.acceleration_x = accelerations[0][0];
    bmx160_data_.acceleration_y = accelerations[0][1];
    bmx160_data_.acceleration_z = accelerations[0][2];
    bmx160_data_.angular_velocity_x = avs[0][0];
    bmx160_data_.angular_velocity_y = avs[0][1];
    bmx160_data_.angular_velocity_z = avs[0][2];
    bmx160_data_.compass_x = mags[0][0];
    bmx160_data_.compass_y = mags[0][1];
    bmx160_data_.compass_z = mags[0][2];
    bmx160_data_.attitude_x = atan2(bmx160_data_.angular_velocity_y, bmx160_data_.angular_velocity_z); // roll calculation
    /* perform calculation in Kalman Filter function directly and update attitude_y there */
    // bmx160_data_.attitude_y = atan2(-acceleration.x, sqrt(acceleration.y*acceleration.y + acceleration.z*acceleration.z)); // pitch calculation
    bmx160_data_.attitude_z = atan2(bmx160_data_.compass_x, bmx160_data_.compass_y); // yaw (heading) calculation

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
    sensors_event_t acceleration; // acceleration values in meters per second squared
    sensors_event_t angular_velocity;
    sensors_event_t temperature; // temperature in degrees Celsius

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

    /* after reading the mpu, populate values in private data struct */
    /* NOTE: need to fix orientation frame later */
    // mpu6050_data_.acceleration_x = acceleration.acceleration.x;
    // mpu6050_data_.acceleration_y = acceleration.acceleration.y;
    // mpu6050_data_.acceleration_z = acceleration.acceleration.z;
    std::vector<std::vector<double>> original_acceleration {
        {acceleration.acceleration.x, acceleration.acceleration.y, acceleration.acceleration.z}
        };
    std::vector<std::vector<double>> original_av {
        {angular_velocity.gyro.roll, angular_velocity.gyro.pitch, angular_velocity.gyro.heading}
    };
    std::vector<std::vector<double>> transform {
        {-1, 0, 0},
        {0, 1, 0},
        {0, 0, -1}
    };
    std::vector<std::vector<double>> accelerations = gemm(original_acceleration, transform);
    std::vector<std::vector<double>> avs = gemm(original_av, transform);
    mpu6050_data_.acceleration_x = original_acceleration[0][0];
    mpu6050_data_.acceleration_y = original_acceleration[0][1];
    mpu6050_data_.acceleration_z = original_acceleration[0][2];
    mpu6050_data_.angular_velocity_x = avs[0][0];
    mpu6050_data_.angular_velocity_y = avs[0][1];
    mpu6050_data_.angular_velocity_z = avs[0][2];
    mpu6050_data_.attitude_x = atan2(mpu6050_data_.acceleration_y, mpu6050_data_.acceleration_z); // roll calculation
    /* perform calculation in Kalman Filter function directly and update attitude_y there */
    // mpu6050_data_.attitude_y = atan2(-acceleration.x, sqrt(acceleration.y*acceleration.y + acceleration.z*acceleration.z)); // pitch calculation
    mpu6050_data_.attitude_z = atan2(bmx160_data_.compass_x, bmx160_data_.compass_y); // only bmx has magnetometer, so just copy yaw from bmx sensor
    mpu6050_data_.temperature = temperature.temperature;

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
    readBMX160();

    /*
     *  Perform initial attitude calculation.
     *  See calculateAttitude function first for details.
     */
    // TODO LAB 5 YOUR CODE HERE.
    bmx160_data_.attitude_y = atan2(-bmx160_data_.acceleration_x, sqrt(bmx160_data_.acceleration_y*bmx160_data_.acceleration_y + bmx160_data_.acceleration_z*bmx160_data_.acceleration_z)); // pitch calculation

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
    readMPU6050();

    /*
     *  Perform initial attitude calculation.
     *  See calculateAttitude function first for details.
     */
    // TODO LAB 5 YOUR CODE HERE.
    mpu6050_data_.attitude_y = atan2(-mpu6050_data_.acceleration_x, sqrt(mpu6050_data_.acceleration_y*mpu6050_data_.acceleration_y + mpu6050_data_.acceleration_z*mpu6050_data_.acceleration_z)); // pitch calculation
    /*
     *  Configure Y attitude Kalman filter.
     */
    mpu6050_kalman_filter_attitude_y_.setAngle(radiansToDegrees(mpu6050_data_.attitude_y));
    mpu6050_kalman_filter_attitude_y_.setQangle(KalmanFilterParameter::q_angle);
    mpu6050_kalman_filter_attitude_y_.setQbias(KalmanFilterParameter::q_bias);
    mpu6050_kalman_filter_attitude_y_.setRmeasure(KalmanFilterParameter::r_measure);
}

void
IMU::calculateAttitudeBMX160()
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
    double attitude_y_raw = atan2(-bmx160_data_.acceleration_x, sqrt(bmx160_data_.acceleration_y*bmx160_data_.acceleration_y + bmx160_data_.acceleration_z*bmx160_data_.acceleration_z)); // pitch calculation
    const double attitude_y_kalman_filter = bmx160_kalman_filter_attitude_y_.getAngle(
            radiansToDegrees(attitude_y_raw), radiansToDegrees(bmx160_data_.angular_velocity_y),
            PeriodParameter::fast);

    /*
     *  Convert the filtered Y attitude data back to radians
     *  using the degreesToRadians function, and populate
     *  the corresponding entry in the member sensor
     *  data struct.
     */
    // TODO LAB 5 YOUR CODE HERE.
    bmx160_data_.attitude_y = degreesToRadians(attitude_y_kalman_filter);
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

    /* https://arduino.stackexchange.com/questions/18625/converting-three-axis-magnetometer-to-degrees#:~:text=heading%20%3D%20atan2(y%2C%20x,(rather%20than%20magnetic)%20heading. */
    /* The above link describes how to calculate the heading from the magnetometer readings. A simple google search shows the reference frame of the MPU sensor, check below results */
    /* https://www.google.com/search?q=mpu+sensor+reference+frame&rlz=1C1CHBF_enUS1007US1007&sxsrf=AJOqlzXa2GW1BA2s5GrHy1SCbkNYakJAvw:1679257122029&source=lnms&tbm=isch&sa=X&ved=2ahUKEwikyvqh6Oj9AhV1I30KHU8aA4kQ_AUoAXoECAEQAw&biw=1920&bih=1049&dpr=1#imgrc=8f3Z7uUsu9p35M */


    /*
     *  Filter the raw Y attitude data using the Kalman filter.
     */
    double attitude_y_raw = atan2(-mpu6050_data_.acceleration_x, sqrt(mpu6050_data_.acceleration_y*mpu6050_data_.acceleration_y + mpu6050_data_.acceleration_z*mpu6050_data_.acceleration_z)); // pitch calculation
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
    mpu6050_data_.attitude_y = degreesToRadians(attitude_y_kalman_filter);
}
}
