/*
 * display.cpp
 *
 *  Created on: Jan 2, 2023
 *      Author: simonyu
 */

#include "display.h"
#include "parameter.h"

namespace biped
{
Display::Display(const unsigned& line, const bool& raw) : line_(line)
{
    if (raw)
    {
        restoreCursor();
    }
    else
    {
        clearLine();
        carriageReturn();
    }
}

Display::~Display()
{
    flush();

    cursor_x_locations_[line_] = sh1107_.getCursorX();
    displayed_ = false;
}

void
Display::initialize()
{
    int16_t text_bound_x = 0;
    int16_t text_bound_y = 0;

    sh1107_.begin(AddressParameter::display, false);
    sh1107_.setCursor(0, 0);
    sh1107_.setRotation(DisplayParameter::rotation_upright);
    sh1107_.setTextSize(DisplayParameter::text_size);
    sh1107_.setTextColor(SH110X_WHITE);
    sh1107_.setTextWrap(false);
    sh1107_.getTextBounds(" ", 0, 0, &text_bound_x, &text_bound_y, &font_width_, &font_height_);
    sh1107_.clearDisplay();
    sh1107_.display();
}

void
Display::display()
{
    if (!displayed_)
    {
        sh1107_.display();
        displayed_ = true;
    }
}

Display&
Display::operator<<(const StreamManipulator& item)
{
    switch (item)
    {
        case StreamManipulator::carriage_return:
        {
            flush();
            carriageReturn();
            break;
        }
        case StreamManipulator::clear_line:
        {
            flush();
            clearLine();
            break;
        }
        case StreamManipulator::endl:
        {
            ss_ << "\n";
            line_ ++;
            break;
        }
        default:
        {
            break;
        }
    }

    return *this;
}

void
Display::carriageReturn()
{
    sh1107_.setCursor(0, line_ * font_height_);
}

void
Display::clearLine()
{
    sh1107_.fillRect(0, line_ * font_height_, DisplayParameter::width, font_height_, 0);
}

void
Display::flush()
{
    int16_t text_bound_x = 0;
    int16_t text_bound_y = 0;
    uint16_t text_height = 0;
    uint16_t text_width = 0;

    sh1107_.getTextBounds(ss_.str().c_str(), sh1107_.getCursorX(), sh1107_.getCursorY(),
            &text_bound_x, &text_bound_y, &text_width, &text_height);
    sh1107_.fillRect(sh1107_.getCursorX(), sh1107_.getCursorY(), text_width, text_height, 0);
    sh1107_.print(ss_.str().c_str());
    ss_.str(std::string());
}

void
Display::restoreCursor()
{
    if (cursor_x_locations_.count(line_))
    {
        sh1107_.setCursor(cursor_x_locations_[line_], line_ * font_height_);
    }
    else
    {
        sh1107_.setCursor(0, line_ * font_height_);
    }
}

std::map<unsigned, int16_t> Display::cursor_x_locations_ = std::map<unsigned, int16_t>();
volatile bool Display::displayed_ = true;
uint16_t Display::font_height_ = 0;
uint16_t Display::font_width_ = 0;
Adafruit_SH1107 Display::sh1107_ = Adafruit_SH1107(DisplayParameter::height,
        DisplayParameter::width, &Wire, DisplayParameter::reset_pin, DisplayParameter::preclk,
        DisplayParameter::postclk);
}
