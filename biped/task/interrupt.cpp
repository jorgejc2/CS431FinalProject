/*
 * interrupt.cpp
 *
 *  Created on: Dec 3, 2022
 *      Author: simonyu
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "common/global.h"
#include "task/interrupt.h"
#include "common/parameter.h"
#include "sensor/sensor.h"

namespace biped
{
void IRAM_ATTR
encoderLeftAInterruptHandler()
{
    /*
     *  Validate Sensor object pointer and
     *  call left encoder callback function.
     *  See the Sensor class for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
}

void IRAM_ATTR
encoderLeftBInterruptHandler()
{
    /*
     *  Validate Sensor object pointer and
     *  call left encoder callback function.
     *  See the Sensor class for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
}

void IRAM_ATTR
encoderRightAInterruptHandler()
{
    /*
     *  Validate Sensor object pointer and
     *  call right encoder callback function.
     *  See the Sensor class for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
}

void IRAM_ATTR
encoderRightBInterruptHandler()
{
    /*
     *  Validate Sensor object pointer and
     *  call right encoder callback function.
     *  See the Sensor class for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
}

void IRAM_ATTR
ioExpanderAInterruptHandler()
{
    if (task_handle_io_expander_a_interrupt_)
    {
        vTaskNotifyGiveFromISR(task_handle_io_expander_a_interrupt_, nullptr);
    }
}

void IRAM_ATTR
ioExpanderBInterruptHandler()
{
    if (task_handle_io_expander_b_interrupt_)
    {
        vTaskNotifyGiveFromISR(task_handle_io_expander_b_interrupt_, nullptr);
    }
}

bool IRAM_ATTR
timerInterruptHandler(void* timer)
{
    if (task_handle_real_time_)
    {
        vTaskNotifyGiveFromISR(task_handle_real_time_, nullptr);
    }

    return true;
}
}
