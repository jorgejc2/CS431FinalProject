/*
 * task.cpp
 *
 *  Created on: Dec 3, 2022
 *      Author: simonyu
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// #include "actuator/actuator.h"
// #include "controller/controller.h"
#include "platform/display.h"
#include "common/global.h"
#include "platform/io_expander.h"
#include "platform/neopixel.h"
// #include "planner/planner.h"
#include "platform/serial.h"
#include "sensor/sensor.h"
#include "task/task.h"

namespace biped
{
void
ioExpanderAInterruptTask(void* pvParameters)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (io_expander_a_)
        {
            io_expander_a_->onInterrupt();
        }
    }

    vTaskDelete(nullptr);
}

void
ioExpanderBInterruptTask(void* pvParameters)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (io_expander_b_)
        {
            io_expander_b_->onInterrupt();
        }
    }

    vTaskDelete(nullptr);
}

void
realTimeTask(void* pvParameters)
{
    unsigned long time_point_start = 0;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        interval_real_time_task_ = micros() - time_point_start;
        time_point_start = micros();

        /*
         *  Perform fast domain sensing.
         *  See the Sensor class for details.
         */
        // TODO LAB 5 YOUR CODE HERE.

        /*
         *  Perform fast domain control.
         *  See the Controller class for details.
         */
        // TODO LAB 7 YOUR CODE HERE.

        /*
         *  Slow domain tasks.
         */
        if (timer_domain_ >= PeriodParameter::slow)
        {
            /*
             *  Perform slow domain sensing.
             *  See the Sensor class for details.
             */
            // TODO LAB 5 YOUR CODE HERE.

            /*
             *  Perform slow domain control.
             *  See the Controller class for details.
             */
            // TODO LAB 7 YOUR CODE HERE.

            /*
             *  Reset timing domain timer.
             */
            timer_domain_ = 0;
        }

        /*
         *  Perform actuation.
         *  See the Actuator class for details.
         */
        // TODO LAB 7 YOUR CODE HERE.

        /*
         *  Update timing domain timer.
         */
        timer_domain_ += PeriodParameter::fast;

        execution_time_real_time_task_ = micros() - time_point_start;
    }

    vTaskDelete(nullptr);
}

void
bestEffortTask()
{
    Display(0) << "Biped: #" << serial_number_;

    Display(1) << "Real-time: " << execution_time_real_time_task_ << " "
            << interval_real_time_task_;

    if (controller_->getActiveStatus())
    {
        Display(2) << "Controller: active";
    }
    else
    {
        Display(2) << "Controller: inactive";
    }

    /*
     *  Execute plan.
     *  See the Planner class for details.
     */
    // TODO LAB 8 YOUR CODE HERE.

    /*
     *  Show the NeoPixel frame.
     *  See the NeoPixel class for details.
     */
    neopixel_->show();

    Display::display();
}
}
