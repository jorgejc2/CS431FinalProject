/**
 *  @file   biped.cpp
 *  @author Simon Yu
 *  @date   01/12/2022
 */

/*
 *  External headers.
 */
#include <EEPROM.h>

/*
 *  Project headers.
 */
#include "pin.h"
#include "neopixel.h"
#include "display.h"
#include "serial.h"

/*
 *  Use biped namespace.
 */
using namespace biped;

unsigned int id;
// TODO Declare variables for the neopixel LEDs
unsigned int counter;

biped::NeoPixel* np;
std::shared_ptr<NeoPixel::Frame> curr_frame;// = std::make_shared<biped::NeoPixel::Frame>();
std::shared_ptr<NeoPixel::Frame> next_frame;// = std::make_shared<biped::NeoPixel::Frame>();


void
setup()
{
    Wire.setPins(ESP32Pin::i2c_sda, ESP32Pin::i2c_scl);
    biped::Serial::setLogLevelMax(SerialParameter::log_level_max);
    Wire.begin();
    EEPROM.begin(EEPROMParameter::size);
    id = static_cast<unsigned int>(EEPROM.read(AddressParameter::eeprom_serial_number));

    // TODO Initialize the display, serial, and neopixel
    Display::initialize();
    biped::Serial::initialize();

    /* setting frame */

    np = new NeoPixel();

    curr_frame = std::make_shared<NeoPixel::Frame>();

    curr_frame->push_back(Eigen::Vector3i(0, 255, 0));
    curr_frame->push_back(Eigen::Vector3i(0, 255, 0));
    curr_frame->push_back(Eigen::Vector3i(0, 255, 0));
    curr_frame->push_back(Eigen::Vector3i(0, 255, 0));

    next_frame = std::make_shared<NeoPixel::Frame>();

    next_frame->push_back(Eigen::Vector3i(255, 0, 0));
    next_frame->push_back(Eigen::Vector3i(255, 0, 0));
    next_frame->push_back(Eigen::Vector3i(255, 0, 0));
    next_frame->push_back(Eigen::Vector3i(255, 0, 0));

    np->setFrame(curr_frame);
    np->show();



    // TODO Display and print to serial the robot ID and net ids of your group
    biped::Serial(LogLevel::info) << id <<"\nyaohuiw2" << "\nrileycw2" << "\njorgejc2\n" << counter;
}

void
loop()
{
    // TODO Generate a simple pattern for the LEDs
    Display(0) << id;
    Display(1) << "yaohuiw2";
    Display(2) << "rileycw2";
    Display(3) << "jorgejc2";
    Display(4) << counter;

    Display::display();

    if((counter % 20) == 0) {
      np->setFrame(next_frame);
      np->show();
    }
    else if ((counter % 10) == 0) {
      np->setFrame(curr_frame);
      np->show();
    }

    biped::Serial(LogLevel::info) << counter;
    

  
    delay(50);

    counter++;

    // TODO Display and print to serial the cycle counts
}
