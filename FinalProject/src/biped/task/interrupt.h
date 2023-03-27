/*
 * interrupt.h
 *
 *  Created on: Dec 3, 2022
 *      Author: simonyu
 */

#ifndef TASK_INTERRUPT_H_
#define TASK_INTERRUPT_H_

#include <esp_attr.h>

namespace biped
{
/**
 *  @brief  Left encoder A interrupt handler.
 *
 *  This function is called when the left encoder
 *  interrupt is raised. The function calls the
 *  corresponding callback member function in
 *  the Sensor class.
 */
void IRAM_ATTR
encoderLeftAInterruptHandler();

/**
 *  @brief  Left encoder B interrupt handler.
 *
 *  This function is called when the left encoder
 *  interrupt is raised. The function calls the
 *  corresponding callback member function in
 *  the Sensor class.
 */
void IRAM_ATTR
encoderLeftBInterruptHandler();

/**
 *  @brief  Right encoder A interrupt handler.
 *
 *  This function is called when the right encoder
 *  interrupt is raised. The function calls the
 *  corresponding callback member function in
 *  the Sensor class.
 */
void IRAM_ATTR
encoderRightAInterruptHandler();

/**
 *  @brief  Right encoder B interrupt handler.
 *
 *  This function is called when the right encoder
 *  interrupt is raised. The function calls the
 *  corresponding callback member function in
 *  the Sensor class.
 */
void IRAM_ATTR
encoderRightBInterruptHandler();

void IRAM_ATTR
ioExpanderAInterruptHandler();

void IRAM_ATTR
ioExpanderBInterruptHandler();

bool IRAM_ATTR
timerInterruptHandler(void* timer);
}

#endif
