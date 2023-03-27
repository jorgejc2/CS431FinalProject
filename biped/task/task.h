/*
 * task.h
 *
 *  Created on: Dec 3, 2022
 *      Author: simonyu
 */

#ifndef TASK_TASK_H_
#define TASK_TASK_H_

namespace biped
{
void
ioExpanderAInterruptTask(void* pvParameters);

void
ioExpanderBInterruptTask(void* pvParameters);

/**
 *  @brief  Real-time task function.
 *
 *  This function performs real-time tasks with
 *  strict timing deadlines. There are two timing
 *  domains, a fast domain and a slow domain. The
 *  function is called by the hardware timer
 *  interrupt handler.
 */
void
realTimeTask(void* pvParameters);

/**
 *  @brief  Best-effort task function.
 *
 *  This function performs best-effort or
 *  non-real-time tasks that do not require
 *  strict timing deadlines. The function is
 *  called by the Arduino loop function. The
 *  best-effort tasks are only run after the
 *  real-time tasks finish during each period.
 */
void
bestEffortTask();
}

#endif
