/*
 * camera.cpp
 *
 *  Created on: Jan 11, 2023
 *      Author: simonyu
 */

#include <esp_camera.h>

#include "platform/camera.h"
#include "common/parameter.h"
#include "common/pin.h"
#include "platform/serial.h"

namespace biped
{
Camera::Camera()
{
    camera_config_t camera_config;

    camera_config.ledc_channel = LEDC_CHANNEL_0;
    camera_config.ledc_timer = LEDC_TIMER_0;
    camera_config.pin_d0 = ESP32Pin::camera_d0;
    camera_config.pin_d1 = ESP32Pin::camera_d1;
    camera_config.pin_d2 = ESP32Pin::camera_d2;
    camera_config.pin_d3 = ESP32Pin::camera_d3;
    camera_config.pin_d4 = ESP32Pin::camera_d4;
    camera_config.pin_d5 = ESP32Pin::camera_d5;
    camera_config.pin_d6 = ESP32Pin::camera_d6;
    camera_config.pin_d7 = ESP32Pin::camera_d7;
    camera_config.pin_xclk = ESP32Pin::camera_xclk;
    camera_config.pin_pclk = ESP32Pin::camera_pclk;
    camera_config.pin_vsync = ESP32Pin::camera_vsync;
    camera_config.pin_href = ESP32Pin::camera_href;
    camera_config.pin_sscb_sda = ESP32Pin::camera_sscb_sda;
    camera_config.pin_sscb_scl = ESP32Pin::camera_sscb_scl;
    camera_config.pin_pwdn = ESP32Pin::camera_pwdn;
    camera_config.pin_reset = ESP32Pin::camera_reset;
    camera_config.xclk_freq_hz = CameraParameter::xclk_frequency;
    camera_config.frame_size = FRAMESIZE_QQVGA;
    camera_config.pixel_format = PIXFORMAT_JPEG;
    camera_config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    camera_config.fb_location = CAMERA_FB_IN_DRAM;
    camera_config.jpeg_quality = CameraParameter::jpeg_quality;
    camera_config.fb_count = CameraParameter::frame_buffer_count;

    if (esp_camera_init(&camera_config) != ESP_OK)
    {
        Serial(LogLevel::error) << "Failed to initialize camera.";
        return;
    }
}
}
