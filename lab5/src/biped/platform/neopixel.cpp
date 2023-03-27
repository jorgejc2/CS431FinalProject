/**
 *  @file   neopixel.cpp
 *  @author Simon Yu
 *  @date   04/01/2022
 *  @brief  NeoPixel class source.
 *
 *  This file implements the NeoPixel class.
 */

/*
 *  Project headers.
 */
#include "common/global.h"
#include "utility/math.h"
#include "platform/neopixel.h"
#include "common/parameter.h"
#include "common/pin.h"
#include "platform/serial.h"

/*
 *  biped namespace.
 */
namespace biped
{
NeoPixel::NeoPixel() : neopixel_(NeoPixelParameter::size, ESP32Pin::neopixel, NEO_GRB + NEO_KHZ800)
{
    neopixel_.begin();

    clear();
    setBrightness(NeoPixelParameter::brightness_max);

    frame_ = std::make_shared<NeoPixel::Frame>();
    frame_->push_back(Eigen::Vector3i(0, 0, 0));
    frame_->push_back(Eigen::Vector3i(0, 0, 0));
    frame_->push_back(Eigen::Vector3i(0, 0, 0));
    frame_->push_back(Eigen::Vector3i(0, 0, 0));

    frame_error_ = std::make_shared<NeoPixel::Frame>();
    frame_error_->push_back(Eigen::Vector3i(255, 0, 0));
    frame_error_->push_back(Eigen::Vector3i(255, 0, 0));
    frame_error_->push_back(Eigen::Vector3i(255, 0, 0));
    frame_error_->push_back(Eigen::Vector3i(255, 0, 0));
}

void
NeoPixel::setBrightness(const int& brightness)
{
    /*
     *  Clamp the brightness value between the minimum
     *  and maximum LED brightness values defined in the
     *  NeoPixelParameter name space.
     */
    neopixel_.setBrightness(
            static_cast<int>(clamp(static_cast<double>(brightness),
                    static_cast<double>(NeoPixelParameter::brightness_min),
                    static_cast<double>(NeoPixelParameter::brightness_max))));
}

void
NeoPixel::setFrame(const std::shared_ptr<Frame> frame)
{
    frame_ = frame;
}

void
NeoPixel::clear()
{
    neopixel_.clear();
    neopixel_.show();
}

void
NeoPixel::show()
{
    std::shared_ptr<Frame> frame = nullptr;

    if (Serial::getLogLevelWorst() <= LogLevel::error)
    {
        frame = frame_error_;
    }
    else
    {
        frame = frame_;
    }

    if (!frame || frame->size() != NeoPixelParameter::size)
    {
        Serial(LogLevel::error) << "Invalid frame.";
        return;
    }

    for (int i = 0; i < NeoPixelParameter::size; i ++)
    {
        neopixel_.setPixelColor(i,
                neopixel_.Color(frame->at(i).x(), frame->at(i).y(), frame->at(i).z()));
    }

    neopixel_.show();
}
}   // namespace biped
