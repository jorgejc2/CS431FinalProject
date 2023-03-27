# Lab 3

In this lab you'll learn about how to interface with the MCP23018 IO expander on the robot.

## Lab Assignment

Your program should:

- Setup the IO expander to receive interrupt from the push buttons, and display the status (activated or not) and the number of times each button has been activated on the display (update on press or release is fine, as long as they are consistent)
- Control the motors so that when button A is pressed, the motors cycle through all 4 movement modes (in no particular order): both forward, both backward, left forward right backward, and left backward right forward

## IO expanders

The IO expanders are crucial in getting the robot working. It's a good opportunity to learn to interface with external hardware and use interrupts. They are Microchip MCP23018's ([datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/22103a.pdf); you only need to focus on sections 1.5 through 1.7), and are controlled over I2C, a form of many serial protocol commonly used in embedded systems. In this lab you will focus on interacting with the expanders themselves, and won't touch any of the technical details of I2C, other than that:

- The IO expanders are located on address `0x20` and `0x27`, and we'll use the one with address `0x20` in this lab;
- Due to implementation details, you ___will not___ be able to use I2C communication inside of interrupts (ask me why if you are interested). Because of this, you should only access I2C inside of the `loop`, and use a `volatile` boolean flag to signal when the access should happen. 

One can control the IO expanders by reading and writing to internal registers over I2C, and they are provided as functions in the `MCP23018` class. The other pin setting functions are left empty for you to implement. You just need to pay attention to setting the value for the `IOCON` register. The pair reading and writing functions expect that the `IOCON` register has bits `BANK` and `SEQOP` are cleared (the default). You also need to set the `MIRROR` bit (as we only have one interrupt pin connected), and you are recommended to set the `INTCC` bit.

After an interrupt is raised, it'll be reflected on the level of the hardware pin`io_expander_a_interrupt`. You can attach an interrupt to that IO pin by using:

```c
attachInterrupt(pin, handler, mode);
```

where `pin` is the IO pin you want the interrupt to be attached to, `handler` the ISR, and `mode` one of `CHANGE`, `FALLING`, `RISING`, `ONHIGH`, or `ONLOW`.

Each pin of the IO expander has an input and an open-drain output driven by a latch (the `OLAT` registers). Note that the output latch can contain a different value than the input. Open-drain outputs mean that the output doesn't directly control the level, but only the state of a MOSFET connected between the pin and ground, thus connecting or disconnecting them. An illustration of this is shown [here](https://commons.wikimedia.org/wiki/File:Animated_open_drain_output.gif#/media/File:Animated_open_drain_output.gif) (open-collector is the same as open-drain, it's just different pin names for MOSFETs and BJTs). The expander does include the pullup resistors, which you can enable/disable through the `GPPU` registers.

In order to practice using the expanders for input and output, you will access the status of the 3 push buttons (located on the left side of the display) through interrupts, and control the motor directions.

### Motor control

Each motor can be controlled by a PWM signal and a direction signal in the H-bridge IC. The whole H-bridge IC can be turned on or off depending on the `motor_enable` pin (high is enable). You can find the definition of all these pins in `pin.h`. You can supply a PWM signal to a motor by using `analogWrite(pin, level)`, where `pin` is one of `motor_left_pwm` or `motor_right_pwm`, and `level` is any number between 0 and 255 (any is fine for this lab).

In order to safely test the motors without having the robot runaway while using the display at the same time, you can set the rotation of the display to be inverted. This way you can place the robot upside down, so the motors aren't touching any surfaces without having the display in the opposite way.

### Button input

The push buttons are active low, as the input pins are pulled up to VCC and the buttons connect them to ground when they are activated. They are located on both port A and B on the IO expander. 

When setting up the interrupts, keep in mind:

- What type of interrupt (level or edge triggered) should you use, and what consequences would the cause?
- Where are the interrupt flags located, and how are the interrupt flags cleared?
