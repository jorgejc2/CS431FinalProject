# Lab 2

In this lab you'll learn about hardware timers and interrupts. 

## Hardware timers

In realtime systems, one would often want to schedule tasks to run at precise time periods, for example to collect sensor data, actuate motors. Using delays like what you did in lab 1 is inaccurate, as the tasks being done may not take the same amount of time each iteration. You could also use some absolute time reference (Arduino provides `micros()` and `millis()` that gives the number of microseconds and milliseconds since boot, not accounting for overflow), but those are typically provided by hardware timers (and in the case of Arduino, driven by hardware timer interrupts).

The ESP32 has 4 hardware timers, split into 2 groups and 2 timers per group. You can find these in the [ESP32 Technical Reference](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf), Section 18 Timer Group. Due to Arduino, you should not use timer group 0.

For the purpose of this lab, you'll interact with the timer registers directly, without any abstraction layers/functions. For code, you can find the definition of the registers in `$ARDUINO_LIB_PATH/packages/esp32/hardware/esp32/$VERSION/tools/sdk/esp32/include/soc/esp32/include/soc/timer_group_reg.h`, where `$ARDUNO_LIB_PATH` is the library path for Arduino IDE (usually `~/.arduino15` on Linux, `~/Library/Arduino` on MacOS), and `$VERSION` is the version of the esp32 arduino package (e.g. `2.0.6`). You can use the `struct`-mapped version of the same registers as a starting point. The definition of that is available in `$ARDUINO_LIB_PATH/packages/esp32/hardware/esp32/$VERSION/tools/sdk/esp32/include/soc/esp32/include/soc/timer_group_struct.h`

Note: The registers listed in `timer_group_reg.h` expand to integers, you need to cast them to `(volatile uint32_t *)` before dereferencing and assigning them.

You should not worry about including any extra headers.

## Interrupts

ESP32 has 2 kinds of interrupts: level and edge triggered interrupts (even for internal/non-IO interrupts). Edge interrupts detects the changes and trigger on the edge transition, and level interrupts trigger whenever the level is high. You should use level interrupts in this lab. The distinction won't matter in this lab, but can cause issues when an interrupt is shared among multiple ISRs, as level interrupts can trigger _after_ the instant the interrupt is set.

For setting up the interrupt handler, use the IDF function [`esp_intr_alloc()`](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/intr_alloc.html#_CPPv414esp_intr_allocii14intr_handler_tPvP13intr_handle_t). The `source` argument should be `ETS_TG1_Tx_LEVEL_INTR_SOURCE` where `x` is the timer number, i.e. `0` or `1`.

## Lab Assignment

Your program should:

- Setup 2 hardware timers to generate interrupts at 200Hz (5ms period) and 25Hz (40ms period) respectively. 
- Toggle the SDA and SCL pins within the interrupts (doesn't matter which corresponds to which; I'll use an oscilloscope to verify timing).
- Delay between 1 to 2ms before the end of the interrupt; use `delayMicroseconds(random(1000, 2000));` for this (to simulate tasks to be done within the interrupt; you shouldn't actually do this in real world).

The second requirement means you wouldn't be able to use the display (as it's driven by I2C) during the demo. You can use the display and/or serial when testing your program, before generating a waveform on the pins. The timer functions `micros()` and `millis()` can be useful in this process (but you shouldn't use them for the requirements).

For toggling the pins, you should use the ESP IDF functions (Arduino functions should work, but in my testing they don't for some reason):

```c
gpio_set_direction(pin, GPIO_MODE_OUTPUT);
gpio_set_pin(pin, 1);
```

Some questions for you to think about (they won't count towards your grades):

- What's the difference between reading/writing the registers using a mapped `struct`ure, and reading/writing the registers directly?
- What does `IRAM_ATTR` do? When should you use it? Note: `DRAM` in the context of ESP32 _is not_ [Dynamic RAM](https://en.wikipedia.org/wiki/Dynamic_random-access_memory).
- What happens if tasks in the interrupt (the `delay`s in this lab) take too long?
