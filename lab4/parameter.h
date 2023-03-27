/**
 *  @file   parameter.h
 *  @author Simon Yu
 *  @date   01/12/2022
 */

#ifndef COMMON_PARAMETER_H_
#define COMMON_PARAMETER_H_

#include <cstddef>
#include <cstdint>

namespace biped
{
enum class StreamManipulator
{
    carriage_return,
    clear_line,
    endl
};

enum class LogLevel
{
    fatal = 0,
    error,
    warn,
    info,
    debug,
    trace
};

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

namespace NeoPixelParameter
{
constexpr uint8_t brightness_max = 30;
constexpr uint8_t brightness_min = 10;
constexpr uint16_t size = 4;
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
