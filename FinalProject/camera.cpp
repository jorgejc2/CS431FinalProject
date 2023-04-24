#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include "common/global.h"
#include "common/type.h"
#include "common/pin.h"
#include "sensor/camera.h"


namespace biped
{
/**
 *  @brief  Camera class.
 *
 */

Camera::Camera(){


    const char* ssid = "**********";
    const char* password = "**********";

    camera_config_t cam_config;
    cam_config.pin_d0 = biped::ESP32Pin::camera_d0;
    cam_config.pin_d1 = biped::ESP32Pin::camera_d1;
    cam_config.pin_d2 = biped::ESP32Pin::camera_d2;
    cam_config.pin_d3 = biped::ESP32Pin::camera_d3;
    cam_config.pin_d4 = biped::ESP32Pin::camera_d4;
    cam_config.pin_d5 = biped::ESP32Pin::camera_d5;
    cam_config.pin_d6 = biped::ESP32Pin::camera_d6;
    cam_config.pin_d7 = biped::ESP32Pin::camera_d7;
    cam_config.pin_xclk = biped::ESP32Pin::camera_xclk;
    cam_config.pin_pclk = biped::ESP32Pin::camera_pclk;
    cam_config.pin_vsync = biped::ESP32Pin::camera_vsync;
    cam_config.pin_href = biped::ESP32Pin::camera_href;
    cam_config.pin_sscb_sda = biped::ESP32Pin::camera_sscb_sda;
    cam_config.pin_sscb_scl = biped::ESP32Pin::camera_sscb_scl;
    cam_config.pin_pwdn = biped::ESP32Pin::camera_pwdn;
    cam_config.pin_reset = biped::ESP32Pin::camera_reset;
    cam_config.xclk_freq_hz = 20000000;
    cam_config.pixel_format = PIXFORMAT_JPEG; // for streaming
    //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
    cam_config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    cam_config.jpeg_quality = 12;
    cam_config.fb_count = 1;
    cam_config.frame_size = FRAMESIZE_SVGA;
    cam_config.fb_location = CAMERA_FB_IN_DRAM;

    // camera init
    esp_err_t err = esp_camera_init(&cam_config);
    if (err != ESP_OK) {
        return;
    }

    sensor_t * s = esp_camera_sensor_get();
    // drop down frame size for higher initial frame rate
    if(cam_config.pixel_format == PIXFORMAT_JPEG){
        s->set_framesize(s, FRAMESIZE_QVGA);
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);

    Serial.println("");
    Serial.println("WiFi connected");

    IPAddress ip;
    ip = WiFi.localIP();

    Serial.print("Camera Ready! Use 'http://");
    Serial.print(WiFi.localIP());
    Serial.println("' to connect");
}
}