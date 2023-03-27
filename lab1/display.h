/*
 * display.h
 *
 *  Created on: Jan 2, 2023
 *      Author: simonyu
 */

#ifndef PLATFORM_DISPLAY_H_
#define PLATFORM_DISPLAY_H_

#include <Adafruit_SH110X.h>
#include <map>
#include <mutex>
#include <sstream>

#include "parameter.h"

namespace biped
{
class Display
{
public:

    Display(const unsigned& line, const bool& raw = false);

    ~Display();

    static void
    initialize();

    static void
    display();

    Display&
    operator<<(const StreamManipulator& item);

    template<typename Type>
    inline Display&
    operator<<(const Type& item)
    {
        ss_ << item;

        return *this;
    }

private:

    void
    carriageReturn();

    void
    clearLine();

    void
    flush();

    void
    restoreCursor();

    static std::map<unsigned, int16_t> cursor_x_locations_;
    static volatile bool displayed_;
    static uint16_t font_height_;
    static uint16_t font_width_;
    unsigned line_;
    static std::unique_lock<std::mutex> lock_sh1107_;
    static std::mutex mutex_sh1107_;
    static Adafruit_SH1107 sh1107_;
    std::stringstream ss_;
};
}

#endif
