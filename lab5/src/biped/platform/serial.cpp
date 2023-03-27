/**
 *  @file   serial.cpp
 *  @author Simon Yu
 *  @date   11/19/2021
 *  @brief  Serial class source.
 *
 *  This file implements the serial class.
 */

#include <Arduino.h>

#include "platform/serial.h"
#include "common/parameter.h"

namespace biped
{
Serial::Serial(const LogLevel& log_level, const bool& raw) : log_level_(log_level), raw_(raw)
{
    if (log_level_ > log_level_max_)
    {
        return;
    }

    if (!raw)
    {
        switch (log_level_)
        {
            case LogLevel::fatal:
            {
                ss_ << "[FATAL]: ";
                break;
            }
            case LogLevel::error:
            {
                ss_ << "[ERROR]: ";
                break;
            }
            case LogLevel::warn:
            {
                ss_ << "[WARN]: ";
                break;
            }
            case LogLevel::info:
            {
                ss_ << "[INFO]: ";
                break;
            }
            case LogLevel::debug:
            {
                ss_ << "[DEBUG]: ";
                break;
            }
            case LogLevel::trace:
            {
                ss_ << "[TRACE]: ";
                break;
            }
            default:
            {
                ss_ << "[FATAL]: ";
                break;
            }
        }
    }
}

Serial::~Serial()
{
    if (log_level_ > log_level_max_)
    {
        return;
    }

    if (raw_)
    {
        ::Serial.print(ss_.str().c_str());
    }
    else
    {
        ::Serial.println(ss_.str().c_str());
    }

    if (log_level_ < log_level_worst_)
    {
        log_level_worst_ = log_level_;
    }
}

void
Serial::initialize()
{
    ::Serial.begin(SerialParameter::baud_rate);
}

LogLevel
Serial::getLogLevelWorst()
{
    return log_level_worst_;
}

void
Serial::setLogLevelMax(const LogLevel& log_level_max)
{
    log_level_max_ = log_level_max;
}

Serial&
Serial::operator<<(const StreamManipulator& item)
{
    if (log_level_ > log_level_max_)
    {
        return *this;
    }

    switch (item)
    {
        case StreamManipulator::carriage_return:
        {
            ss_ << "\r";
            break;
        }
        case StreamManipulator::endl:
        {
            ss_ << "\r\n";
            break;
        }
        default:
        {
            break;
        }
    }

    return *this;
}

LogLevel Serial::log_level_max_ = LogLevel::trace;
LogLevel Serial::log_level_worst_ = LogLevel::trace;
}
