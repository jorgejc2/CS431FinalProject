# Lab 1

In this lab you'll setup the development environment and get to know the basic interfaces.

## Setup

We'll be using Arduino IDE v2 for the labs. You can install it by visiting the [Arduino website](https://www.arduino.cc/en/software).

If you are doing this on the lab machines, you should use the [AppImage link](https://downloads.arduino.cc/arduino-ide/arduino-ide_2.0.3_Linux_64bit.AppImage).

If you prefer a CLI development experience, you can use the [Arduino CLI](https://arduino.github.io/arduino-cli/0.30/installation/).

### Board setup

You'll need to setup the ESP32 toolchain in Arduino IDE. Open the "Preferences" menu in Arduino IDE (or use `ctrl-,`), and at the bottom of the menu you should see the option "Additional boards manager URLs". Add this URL to that list:

```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

Then click "OK" to save the settings, and go to the "Boards Manager" (accessible from the left bar) and install the "esp32" boards. This should download and install the ESP32 toolchain.

Finally, in the "Tools" menu, select "Board" > "esp32" and select "ESP32 Dev Module".

When you use the serial monitor, remember to set the baud rate to 115200. Also, always remember to power on the robot before you flash it.

### Libraries

We'll need to use several libraries for this lab. You can install them by using the library manager in the Arduino IDE (accessible from the left bar). Make sure to install the version of the libraries listed above for the sake of compatibility. The libraries are as follows:

- Adafruit NeoPixel 1.10.4
- Adafruit SH110X 2.1.8
- Adafruit BusIO 1.14.0
- Adafruit GFX Library 1.11.3
- Eigen 0.2.3

The robot also depends on several more libraries. We won't need them for lab 1, but it is advised that you install them as soon as possible:

- Adafruit MPU6050 2.0.6
- Adafruit Unified Sensor 1.1.6
- Kalman Filter Library 1.0.2
- DFRobot BMX160 1.0.1

## Lab Assignment

For the lab assignment, you'll learn how to interface with the display, LEDs, and the serial.

Your program should:

- Display the robot ID and the net ids of your group members on the display, one item on each line
- Display a cycle counter on the next line in the display. The counter should increment once every roughly 50ms (you can just delay 50ms in your loop; don't worry about overflows).
- Like for the display, print the robot ID and net ids of your group members to the serial, followed by the cycle counts.
- Display a simple changing pattern on the LEDs. You can do this in anyway you want (just don't make it flicker too fast so people don't get epilepsy :) )

## Provided Utilities

Here we provide a brief overview of the interface for controlling the display, serial, and LEDs. When in doubt, consult the source code and/or ask the TA.

### Serial

We provided a class `biped::Serial` to facilitate logging. After initialization, you can pass the log level as an argument, and use the `<<` streaming operators to pass in the content (just like `cout`). Example: `Serial(LogLevel::info) << "hello";` prints "hello" with a log level of info.

Under the hood, this is just a wrapper over the `Serial` that Arduino provides, but we do advise that you use this as it provides some helpful features:

- It's easier to log different values onto the same line;
- It's easy to log with different logging levels, and to filter them when needed;
- It keeps track of the worst log level, and when that reaches the error level, displays an obvious red color on the LEDs.

### NeoPixel RGB LEDs

There are 4 NeoPixel RGB LEDs around the robot. Note that these LEDs work by communicating over a serial line, and are thus not as realtime as normal LEDs.

We provide a `NeoPixel` class for interfacing with them. A `Frame` object is used to represent the color information on all of the LEDs. Use `setFrame` on the NeoPixel object to set the colors and use `show` to send the information to the LEDs. These 2 operations are separated because sending the colors take time (if you are interested, you can look up the datasheet (the LEDs are WS2812B) and calculate how much time it'll take).

As mentioned for the `Serial` class, the LEDs will instead show red on all LEDs if there's a error in the log.

### Display

The `Display` class is structured very similar to the `Serial` class, as both can be very helpful in debugging your code. Instead of the log level, `Display` takes a line number to display the information at. Example: `Display(3) << "hello";` displays "hello" at the beginning of line 3 (line number starts from 0). Similar to the NeoPixels, it also takes time to flush the display contents. Hence, you need to use `display()` to flush the display.
