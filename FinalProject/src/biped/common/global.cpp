/*
 * global.cpp
 *
 *  Created on: Dec 1, 2022
 *      Author: simonyu
 */

#include "common/global.h"

namespace biped
{
std::shared_ptr<IOExpander> io_expander_a_ = nullptr;
std::shared_ptr<IOExpander> io_expander_b_ = nullptr;
std::shared_ptr<NeoPixel> neopixel_ = nullptr;
std::shared_ptr<Sensor> sensor_ = nullptr;

std::mutex mutex_wire_;
std::unique_lock<std::mutex> lock_wire_ = std::unique_lock<std::mutex>(mutex_wire_,
        std::defer_lock);

TaskHandle_t task_handle_io_expander_a_interrupt_ = nullptr;
TaskHandle_t task_handle_io_expander_b_interrupt_ = nullptr;
TaskHandle_t task_handle_real_time_ = nullptr;

unsigned long execution_time_real_time_task_ = 0;
unsigned long interval_real_time_task_ = 0;
double timer_domain_ = 0;

unsigned serial_number_ = 0;

int biped_direction_ = 0;
}
