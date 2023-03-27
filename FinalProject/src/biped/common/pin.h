/**
 *  @file   pin.h
 *  @author Simon Yu
 *  @date   04/01/2022
 *  @brief  Pin header.
 *
 *  This file defines pin-specific classes,
 *  structs, or enums.
 */

/*
 *  Include guard.
 */
#ifndef COMMON_PIN_H_
#define COMMON_PIN_H_

#include <cstdint>

/*
 *  biped namespace.
 */
namespace biped
{
namespace ESP32Pin
{
constexpr int camera_d0 = 19;
constexpr int camera_d1 = 22;
constexpr int camera_d2 = 23;
constexpr int camera_d3 = 21;
constexpr int camera_d4 = 18;
constexpr int camera_d5 = 17;
constexpr int camera_d6 = 16;
constexpr int camera_d7 = 2;
constexpr int camera_href = 15;
constexpr int camera_pclk = 5;
constexpr int camera_pwdn = -1;
constexpr int camera_reset = -1;
constexpr int camera_sscb_scl = 26;
constexpr int camera_sscb_sda = 14;
constexpr int camera_vsync = 13;
constexpr int camera_xclk = 4;
constexpr uint8_t io_expander_a_interrupt = 25;
constexpr uint8_t io_expander_b_interrupt = 33;
constexpr uint8_t i2c_scl = 26;
constexpr uint8_t i2c_sda = 14;
constexpr uint8_t motor_left_encoder_a = 39;    //!< (Input) Left motor encoder A.
constexpr uint8_t motor_left_encoder_b = 36;    //!< (Input) Left motor encoder B.
constexpr uint8_t motor_right_encoder_a = 35;   //!< (Input) Right motor encoder A.
constexpr uint8_t motor_right_encoder_b = 34;   //!< (Input) Right motor encoder B.
constexpr uint8_t motor_left_pwm = 27;  //!< (Output) Left motor PWM value.
constexpr uint8_t motor_right_pwm = 12; //!< (Output) Right motor PWM value.
constexpr uint8_t neopixel = 32;    //!< (Output) NeoPixel LED array.
}

namespace IOExpanderAPortAPin
{
constexpr uint8_t motor_left_direction = 4; //!< (Output) Left motor direction.
constexpr uint8_t motor_right_direction = 3;  //!< (Output) Right motor direction.
constexpr uint8_t mpu6050_interrupt = 5;    //!< (Output) Left motor direction.
constexpr uint8_t pushbutton_a = 6; //!< (Input) Pushbutton A.
constexpr uint8_t pushbutton_b = 7; //!< (Input) Pushbutton B.
constexpr uint8_t time_of_flight_left_interrupt = 1;
constexpr uint8_t time_of_flight_left_shutdown = 0;
}

namespace IOExpanderAPortBPin
{
constexpr uint8_t bmx160_compass_interrupt_a = 3;
constexpr uint8_t bmx160_compass_interrupt_b = 4;
constexpr uint8_t motor_enable = 5;    //!< (Output) Motor enable.
constexpr uint8_t pushbutton_c = 0;    //!< (Input) Pushbutton C.
constexpr uint8_t time_of_flight_middle_interrupt = 6;
constexpr uint8_t time_of_flight_middle_shutdown = 7;
constexpr uint8_t time_of_flight_right_interrupt = 2;
constexpr uint8_t time_of_flight_right_shutdown = 1;
}

namespace IOExpanderBPortAPin
{
}

namespace IOExpanderBPortBPin
{
}
}   // namespace biped

#endif  // COMMON_PIN_H_
