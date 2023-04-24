/**
 *  @file   interrupt.cpp
 *  @author Simon Yu
 *  @date   12/03/2022
 *  @brief  Interrupt function source.
 *
 *  This file implements the interrupt functions.
 */

/*
 *  External headers.
 */
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <gpio_struct.h>
#include <hal/misc.h>

/*
 *  Project headers.
 */
#include "common/global.h"
#include "task/interrupt.h"
#include "common/parameter.h"
#include "planner/planner.h"
#include "sensor/sensor.h"
#include "platform/serial.h"

/* TEMP */
#include "common/pin.h"

/*
 *  Biped namespace.
 */
namespace biped
{
void
attachInterrupt(const uint8_t& pin, void
(*handler)(void), const int& mode)
{
    /*
     *  Declare ESP error status.
     */
    esp_err_t result;

    /*
     *  Attach the given interrupt handler to the given pin.
     */
    result = gpio_isr_handler_add(static_cast<gpio_num_t>(pin),
            reinterpret_cast<gpio_isr_t>(handler), nullptr);

    /*
     *  Log error message to serial and return if failed.
     */
    if (result != ESP_OK)
    {
        Serial(LogLevel::error) << "Failed to add interrupt handler.";
        return;
    }

    /*
     *  Set interrupt type based on the given mode.
     */
    switch (mode)
    {
        case RISING:
        {
            result = gpio_set_intr_type(static_cast<gpio_num_t>(pin), GPIO_INTR_POSEDGE);
            break;
        }
        case FALLING:
        {
            result = gpio_set_intr_type(static_cast<gpio_num_t>(pin), GPIO_INTR_NEGEDGE);
            break;
        }
        case CHANGE:
        {
            result = gpio_set_intr_type(static_cast<gpio_num_t>(pin), GPIO_INTR_ANYEDGE);
            break;
        }
        case ONLOW:
        {
            result = gpio_set_intr_type(static_cast<gpio_num_t>(pin), GPIO_INTR_LOW_LEVEL);
            break;
        }
        case ONHIGH:
        {
            result = gpio_set_intr_type(static_cast<gpio_num_t>(pin), GPIO_INTR_HIGH_LEVEL);
            break;
        }
        case ONLOW_WE:
        {
            result = gpio_set_intr_type(static_cast<gpio_num_t>(pin), GPIO_INTR_LOW_LEVEL);
            break;
        }
        case ONHIGH_WE:
        {
            result = gpio_set_intr_type(static_cast<gpio_num_t>(pin), GPIO_INTR_HIGH_LEVEL);
            break;
        }
        default:
        {
            /*
             *  Unknown interrupt mode. Log error message to serial.
             */
            Serial(LogLevel::error) << "Unknown interrupt mode: " << mode << ".";
            break;
        }
    }

    /*
     *  Log error message to serial if failed.
     */
    if (result != ESP_OK)
    {
        Serial(LogLevel::error) << "Failed to set interrupt mode.";
    }
}

int IRAM_ATTR
digitalReadFromISR(uint8_t pin)
{
    /*
     *  static_cast the given pin into 32-bit unsigned integer.
     */
    uint32_t pin_gpio = static_cast<uint32_t>(pin);

    /*
     *  Read the given GPIO pin and return the pin state read.
     */
    if (pin_gpio < 32)
    {
        return (GPIO.in >> pin_gpio) & 0x1;
    }
    else
    {
        return (HAL_FORCE_READ_U32_REG_FIELD(GPIO.in1, data) >> (pin_gpio - 32)) & 0x1;
    }
}

void IRAM_ATTR
encoderLeftAInterruptHandler()
{
    /*
     *  Validate sensor object pointer and
     *  call left encoder A callback function.
     *  See the sensor class for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    if (sensor_ != nullptr)
        sensor_->onEncoderLeftA();
}

void IRAM_ATTR
encoderLeftBInterruptHandler()
{
    /*
     *  Validate sensor object pointer and
     *  call left encoder B callback function.
     *  See the sensor class for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    if (sensor_ != nullptr)
        sensor_->onEncoderLeftB();
}

void IRAM_ATTR
encoderRightAInterruptHandler()
{
    /*
     *  Validate sensor object pointer and
     *  call right encoder A callback function.
     *  See the sensor class for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    if (sensor_ != nullptr)
        sensor_->onEncoderRightA();
}

void IRAM_ATTR
encoderRightBInterruptHandler()
{
    /*
     *  Validate sensor object pointer and
     *  call right encoder B callback function.
     *  See the sensor class for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    if (sensor_ != nullptr)
        sensor_->onEncoderRightB();
}

void IRAM_ATTR
ioExpanderAInterruptHandler()
{
    /*
     *  Validate I/O expander A interrupt task handle and wake
     *  the I/O expander A interrupt task using the FreeRTOS
     *  vTaskNotifyGiveFromISR function.
     *  See the global and task header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    // if (task_handle_io_expander_a_interrupt_ != nullptr) {
        // biped::Serial(LogLevel::info) << "Enterring io_a interrupt handler";
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(task_handle_io_expander_a_interrupt_, nullptr);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    // }
}

void IRAM_ATTR
ioExpanderBInterruptHandler()
{
    /*
     *  Validate I/O expander B interrupt task handle and wake
     *  the I/O expander B interrupt task using the FreeRTOS
     *  vTaskNotifyGiveFromISR function.
     *  See the global and task header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    if (task_handle_io_expander_b_interrupt_ != nullptr) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(task_handle_io_expander_b_interrupt_, nullptr);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void IRAM_ATTR
pushButtonAInterruptHandler()
{
    /*
     *  Validate planner object pointer and start the plan.
     *  See the planner class for details.
     */
    // TODO LAB 8 YOUR CODE HERE.
    // biped::Serial(LogLevel::info) << "Enterring pushbutton a interrupt";
    if(planner_ != nullptr){
        planner_->start();
    }
}

/* newly created */
void IRAM_ATTR
pushButtonCInterruptHandler()
{
    /*
     *  Validate planner object pointer and start the plan.
     *  See the planner class for details.
     */
    // TODO LAB 8 YOUR CODE HERE.
    if(planner_ != nullptr){
        planner_->start();
    }
}

void IRAM_ATTR
pushButtonBInterruptHandler()
{
    /*
     *  Validate sensor object pointer and call push
     *  button B callback function.
     *  See the sensor class for details.
     */
    // TODO LAB 7 YOUR CODE HERE.
    push_button_b_pressed++;
    if (sensor_ != nullptr) {
        sensor_->onPushButtonB();
    }
}

bool IRAM_ATTR
timerInterruptHandler(void* timer)
{
    /*
     *  Validate real-time task handle and wake the real-time
     *  task using the FreeRTOS vTaskNotifyGiveFromISR function.
     *  See the global and task header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    if (task_handle_real_time_ != nullptr) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(task_handle_real_time_, nullptr);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    return true;
}
}   // namespace biped
