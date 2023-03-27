# Lab 5

From this lab onwards, you'll be building towards making your robot balance on its own, and perform interesting maneuvers. You'll be implementing the sensing portion in this lab and the next lab. 

## Lab Assignment

- Read the linear acceleration along each axis, angular velocity around each axis, and yaw angle (around Z axis), every 5ms (aka `PeriodParameter::fast` seconds).
- Display the linear acceleration, angular velocity, and yaw angle, according to the standard body model. 
- Read the distance from each time-of-flight sensor, every 40ms (aka `PeriodParameter::slow` seconds).
- Display the distance from each time-of-flight sensor.

## Build System

We'll switch to CMake for this and future labs. Use either the lab machines or a Linux/Mac laptop. Run the `scripts/biped/setup.bash` within the repo to set things up. Then, `make` in the `build/biped` directory, and use `make upload SERIAL_PORT=<port>` to flash the robot.

## Attitude sensing

We'll use the MPU6050 for sensing attitude. Note that the data returned by the sensor can be from a different reference frame, and you should convert it to the [standard body frame](https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-refframes).

For the yaw angle, read the magnetometer readings along X and Y, and get the angle from that. Note that won't be an absolute heading, as that'll require more complicated calibration. If the angle changes in a normal rate and doesn't drift much, it's fine.

## Distance sensing

We'll use the VL53L4CX sensors for sensing distance. These sensors require the use of a `SHUTDOWN` pin when setting up, and those pins live on the IO expanders. We have included utilities for interacting with them (which is very similar to what you implemented in lab 3, but with generic interrupt handling. Don't worry if the sensors don't return valid data all the time.
