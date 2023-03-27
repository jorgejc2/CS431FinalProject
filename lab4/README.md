# Lab 4

In this lab you'll learn about FreeRTOS tasks and the various synchronization primitives. This lab will build up on lab 3.

## Lab Assignment

The requirements will build up on the lab 3 requirements, and they are repeated here for your convenience:
- Setup the IO expander to receive interrupt from the push buttons, and display the status (activated or not) and the number of times each button has been activated on the display (update on press or release is fine, as long as they are consistent)
- Control the motors so that when button A is pressed, the motors cycle through all 4 movement modes (in no particular order): both forward, both backward, left forward right backward, and left backward right forward

The requirements specific to lab 4 are:
- create different FreeRTOS tasks for
  1. handling buttons presses/releases;
  2. actuating the motors properly;
  3. displaying the button actuatiion status and counts.
- use the appropriate types of locks to synchronize among tasks, and between tasks and interrupt(s)

## FreeRTOS tasks

Tasks are a good way to organize different work components in embedded systems. Concept-wise they are similar to threads. FreeRTOS tasks have priorities and implement preemptive scheduling using time slicing. This means every once in a while the scheduler will interrupt the current running task, and switch to the task with the highest priority and available to run. You can read more about FreeRTOS tasks [here](https://www.freertos.org/taskandcr.html).

You can create a task by using the [`xTaskCreate` function](https://www.freertos.org/a00125.html). There are some useful task parameters you can use in `parameter.h`.

## FreeRTOS synchronization primitives

The 2 main synchronization primitives in FreeRTOS are semaphores and mutexes. You may remember them from your OS class. The general concepts will carry over pretty well, but the implementation and API may differ. You can learn more about FreeRTOS semaphores and mutexes [here](https://www.freertos.org/xSemaphoreCreateBinary.html).

PS: yes there's the note that says it would be better to use task notifications than semaphores. However, given that the APIs are very similar and semaphores (the name and concept) are pretty general, we'll stick with semaphores for this lab.
