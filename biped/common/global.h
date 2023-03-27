/*
 * global.h
 *
 *  Created on: Dec 1, 2022
 *      Author: simonyu
 */

#ifndef COMMON_GLOBAL_H_
#define COMMON_GLOBAL_H_

#include <freertos/FreeRTOS.h>
#include <memory>
#include <mutex>
#include <freertos/task.h>

namespace biped
{
class Actuator;
class Camera;
class Controller;
class IOExpander;
class NeoPixel;
class Planner;
class Sensor;

extern std::shared_ptr<Actuator> actuator_;
extern std::shared_ptr<Camera> camera_;
extern std::shared_ptr<Controller> controller_;
extern std::shared_ptr<IOExpander> io_expander_a_;
extern std::shared_ptr<IOExpander> io_expander_b_;
extern std::shared_ptr<NeoPixel> neopixel_;
extern std::shared_ptr<Planner> planner_;
extern std::shared_ptr<Sensor> sensor_;

extern std::mutex mutex_wire_;
extern std::unique_lock<std::mutex> lock_wire_;

extern TaskHandle_t task_handle_io_expander_a_interrupt_;
extern TaskHandle_t task_handle_io_expander_b_interrupt_;
extern TaskHandle_t task_handle_real_time_;

extern unsigned long execution_time_real_time_task_;
extern unsigned long interval_real_time_task_;
extern double timer_domain_;

extern unsigned serial_number_;
}

#endif
