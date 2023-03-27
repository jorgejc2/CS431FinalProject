/**
 *  @file   serial.h
 *  @author Simon Yu
 *  @date   11/19/2021
 *  @brief  Serial class header.
 *
 *  This file defines the serial class.
 */

#ifndef PLATFORM_SERIAL_H_
#define PLATFORM_SERIAL_H_

#include <sstream>

#include "parameter.h"

namespace biped
{
class Serial
{
public:

    Serial(const LogLevel& log_level, const bool& raw = false);

    ~Serial();

    static void
    initialize();

    static LogLevel
    getLogLevelWorst();

    static void
    setLogLevelMax(const LogLevel& log_level_max);

    Serial&
    operator<<(const StreamManipulator& item);

    template<typename Type>
    inline Serial&
    operator<<(const Type& item)
    {
        if (log_level_ > log_level_max_)
        {
            return *this;
        }

        ss_ << item;

        return *this;
    }

private:

    const LogLevel log_level_;
    static LogLevel log_level_max_;
    static LogLevel log_level_worst_;
    const bool raw_;
    std::stringstream ss_;
};
}

#endif
