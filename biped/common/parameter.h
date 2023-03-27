/**
 *  @file   parameter.h
 *  @author Simon Yu
 *  @date   01/12/2022
 */

#ifndef COMMON_PARAMETER_H_
#define COMMON_PARAMETER_H_

#include <cstddef>
#include <cstdint>
#include <freertos/FreeRTOS.h>

#include "common/type.h"

namespace biped
{
namespace AddressParameter
{
constexpr uint8_t display = 0x3C;
constexpr uint8_t eeprom_serial_number = 0x00;
constexpr uint8_t imu_mpu6050 = 0x69;
constexpr uint8_t io_expander_a = 0x00;
constexpr uint8_t io_expander_b = 0x07;
constexpr uint8_t time_of_flight_left = 0x40;
constexpr uint8_t time_of_flight_middle = 0x41;
constexpr uint8_t time_of_flight_right = 0x42;
}

namespace CameraParameter
{
constexpr size_t frame_buffer_count = 2;
constexpr int jpeg_quality = 12;
constexpr int xclk_frequency = 20000000;
}

namespace ControllerParameter
{
constexpr double attitude_y_active = 20;
}

namespace DisplayParameter
{
constexpr double acceleration_z_rotation = 0;
constexpr uint16_t height = 64;
constexpr double low_pass_filter_beta = 0.9;
constexpr uint32_t postclk = 1000000;
constexpr uint32_t preclk = 1000000;
constexpr int8_t reset_pin = -1;
constexpr uint8_t rotation_upright = 1;
constexpr uint8_t rotation_inverted = 3;
constexpr uint8_t text_size = 1;
constexpr uint16_t width = 128;
}

namespace EEPROMParameter
{
constexpr size_t size = 4;
}

namespace EncoderParameter
{
constexpr double low_pass_filter_beta = 0.7;
constexpr unsigned steps_per_meter = 7400;
}

namespace IOExpanderParameter
{
constexpr size_t num_port_pins = 8;
constexpr size_t num_ports = 2;
}

namespace KalmanFilterParameter
{
constexpr float q_angle = 0.001;
constexpr float q_bias = 0.005;
constexpr float r_measure = 0.5;
}

namespace MotorParameter
{
constexpr unsigned pwm_min = 0;
constexpr unsigned pwm_max = 255;
}

namespace NeoPixelParameter
{
constexpr uint8_t brightness_max = 30;
constexpr uint8_t brightness_min = 10;
constexpr uint16_t size = 4;
}

namespace PeriodParameter
{
constexpr double fast = 0.005;
constexpr double slow = 0.04;
}

namespace SerialParameter
{
constexpr unsigned long baud_rate = 115200;
constexpr LogLevel log_level_max = LogLevel::trace;
}

namespace TaskParameter
{
constexpr int core_0 = 0;
constexpr int core_1 = 1;
constexpr unsigned int priority_min = 0;
constexpr unsigned int priority_max = configMAX_PRIORITIES;
constexpr uint32_t stack_size = 4096;
}
}

#endif
