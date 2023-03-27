/**
 *  @file   neopixel.h
 *  @author Simon Yu
 *  @date   04/01/2022
 *  @brief  NeoPixel class header.
 *
 *  This file defines the NeoPixel class.
 */

/*
 *  Include guard.
 */
#ifndef PLATFORM_NEOPIXEL_H_
#define PLATFORM_NEOPIXEL_H_

/*
 *  External headers.
 */
#include <Adafruit_NeoPixel.h>
#include <ArduinoEigenDense.h>
#include <memory>
#include <vector>

/*
 *  biped namespace.
 */
namespace biped
{
/**
 *  @brief  NeoPixel class.
 *
 *  This class provides functions for
 *  controlling the NeoPixel LED array.
 */
class NeoPixel
{
public:

    using Frame = std::vector<Eigen::Vector3i>;

    NeoPixel();

    void
    setBrightness(const int& brightness);

    void
    setFrame(const std::shared_ptr<Frame> frame);

    void
    clear();

    void
    show();

private:

    std::shared_ptr<Frame> frame_;
    std::shared_ptr<Frame> frame_error_;
    Adafruit_NeoPixel neopixel_;
};
}   // namespace biped

#endif  // PLATFORM_NEOPIXEL_H_
