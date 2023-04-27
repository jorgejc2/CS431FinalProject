/**
 *  @file   global.cpp
 *  @author Simon Yu
 *  @date   12/01/2022
 *  @brief  Global variable source.
 *
 *  This file initializes the global variables.
 */

/*
 *  Project headers.
 */
#include "common/global.h"
#include "common/type.h"

/*
 *  Biped namespace.
 */
namespace biped
{
/*
 *  Initialize object shared pointers to null pointers.
 */
std::shared_ptr<Actuator> actuator_ = nullptr;
std::shared_ptr<Camera> camera_ = nullptr;
std::shared_ptr<Controller> controller_ = nullptr;
std::shared_ptr<IOExpander> io_expander_a_ = nullptr;
std::shared_ptr<IOExpander> io_expander_b_ = nullptr;
std::shared_ptr<NeoPixel> neopixel_ = nullptr;
std::shared_ptr<Planner> planner_ = nullptr;
std::shared_ptr<Sensor> sensor_ = nullptr;
std::shared_ptr<ESP32TimerInterrupt> timer_ = nullptr;

// std::shared_ptr<ControllerReference> controled_reference = nullptr;

/* debugger values */
int output_position_x = 0;
int output_attitude_y = 0;
int volatile push_button_b_pressed = 0;

float integrated_angle_x = 0;
float previous_angle_x = 0;
/*
 *  Initialize I2C driver object mutex and mutex lock.
 */
std::mutex mutex_wire_;
std::unique_lock<std::mutex> lock_wire_ = std::unique_lock<std::mutex>(mutex_wire_,
        std::defer_lock);

/*
 *  Initialize task handles to null pointers.
 */
TaskHandle_t task_handle_camera_ = nullptr;
TaskHandle_t task_handle_io_expander_a_interrupt_ = nullptr;
TaskHandle_t task_handle_io_expander_b_interrupt_ = nullptr;
TaskHandle_t task_handle_real_time_ = nullptr;
TaskHandle_t task_handle_wifi_ = nullptr;

/*
 *  Initialize execution timers to 0.
 */
unsigned long execution_time_real_time_task_ = 0;
unsigned long interval_real_time_task_ = 0;
double timer_domain_ = 0;

/*
 *  Initialize serial number to 0.
 */
unsigned serial_number_ = 0;

int biped_direction_ = 0;
}   // namespace biped
