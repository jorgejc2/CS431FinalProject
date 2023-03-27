/**
 *  @file   biped.cpp
 *  @author Simon Yu
 *  @date   01/12/2022
 */

/*
 *  External headers.
 */
#include <EEPROM.h>

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "driver/gpio.h"

/*
 *  Project headers.
 */
#include "pin.h"
#include "neopixel.h"
#include "display.h"
#include "serial.h"
#include "MCP23018.h"

/*
 *  Use biped namespace.
 */
using namespace biped;

unsigned int id;

// TODO create IO expander instance
MCP23018 * mcp; 
volatile bool read_i2c = false;
bool push_button_a_status = false;
bool push_button_b_status = false;
bool push_button_c_status = false;
int motor_configuration = 0; // 0: left and right forward, 1: left forward, right backward, 2: left backward, right forward, 3: both backward

int a_counter = 0;
int b_counter = 0;
int c_counter = 0;

// TODO declare execution flag(s)
/* from documentation:
The Interrupt Clearing Control (INTCC) configures how
interrupts are cleared. When set (INTCC = 1), the
interrupt is cleared when the INTCAP register is read.
When cleared (INTCC = 0), the interrupt is cleared
when the GPIO register is read.
*/
void gpio_isr () {
  /* Raise flag signaling to read the i2c registers. Due to 'implemenation details', registers should be read in loop and not here.*/
  read_i2c = true;
  return;
};

void
setup()
{
  Wire.setPins(ESP32Pin::i2c_sda, ESP32Pin::i2c_scl);
  biped::Serial::setLogLevelMax(SerialParameter::log_level_max);
  Wire.begin();
  EEPROM.begin(EEPROMParameter::size);
  id = static_cast<unsigned int>(EEPROM.read(AddressParameter::eeprom_serial_number));
  Display::initialize();
  biped::Serial::initialize();

  // TODO setup IO expander
  
  /* The class constructor only accepts 3 bits because whoever created the class already knew the I/O 
  ** expander started at address 0x20. Thus they take three bits and 'or' it with 0x20 to access the 
  ** correct IO expander. In otherwords, you can either send 0x20/0x27, or simply 0/7.
  */
  mcp = new MCP23018(0x20);

  mcp->begin();

  /* set MIRROR and INTCC. The rest, espeically BANK and SEQOP, should be cleared */
  mcp->writeToRegister(IOCON, IOCON_MIRROR | IOCON_INTCC | IOCON_INTPOL);


  // TODO setup IO expnader interrupts
  // attachInterrupt(io_expander_a_interrupt, gpio_isr, RISING);

  // TODO enable motor and set motor speed
  /* By default, all the pins are set up as outputs. Want to set up push buttons as inputs. Low is the same as setting as output */
  mcp->SetDirections(BIT(IOExpanderAPortAPin::pushbutton_a) | BIT(IOExpanderAPortAPin::pushbutton_b), BIT(IOExpanderAPortBPin::pushbutton_c));
  /* set pull up resistor for every pin */
  mcp->SetPullups(0xFF, 0xFF);
  /* enable the motor */
  mcp->SetBPin(IOExpanderAPortBPin::motor_enable, 1);

  /* interrupt configuration */


  /* Page 21 of documentation: by using the DEFVAL register as the interrupt on change source, and interrupt occures
  ** if the pin is the opposite value to its corresponding pin in the DEFVAL register. When the button is not being pressed,
  ** its GPIO pin will be high. Thus for the pins where the push buttons are connected, we want their values in DEFVAL to be high
  ** so that an interrupt occurs when the input is low (pushbuttons are active-low).
  */
  mcp->writeToRegister(DEFVALA, bit(IOExpanderAPortAPin::pushbutton_a) | bit(IOExpanderAPortAPin::pushbutton_b));
  mcp->writeToRegister(DEFVALB, bit(IOExpanderAPortBPin::pushbutton_c));

  /* Page 22 of documentation: set interrupt on change source to be from comparing the input pins to the values in the 
  ** DEFVAL register, not by their previous state. 
  */
  // mcp->writeToRegister(INTCONA, bit(IOExpanderAPortAPin::pushbutton_a) | bit(IOExpanderAPortAPin::pushbutton_b));
  // mcp->writeToRegister(INTCONB, bit(IOExpanderAPortBPin::pushbutton_c));
  attachInterrupt(ESP32Pin::io_expander_a_interrupt, gpio_isr, RISING);


  /* enable interrupt on change from the push buttons */
  mcp->writeToRegister(INTENA, bit(IOExpanderAPortAPin::pushbutton_a) | bit(IOExpanderAPortAPin::pushbutton_b));
  mcp->writeToRegister(INTENB, bit(IOExpanderAPortBPin::pushbutton_c));

  /* set up pin to command left and right motors with pwm signals */
  pinMode(ESP32Pin::motor_left_pwm, OUTPUT);
  analogWrite(ESP32Pin::motor_left_pwm, 200);
  pinMode(ESP32Pin::motor_right_pwm, OUTPUT);
  analogWrite(ESP32Pin::motor_right_pwm, 200);
    
}

void
loop()
{

  int a_pressed = 0;
  int b_pressed = 0;
  int c_pressed = 0;
  // TODO read IO expander and update display
  if (read_i2c) {
    /* due to 'implemenation details', i2c communication must happen in this loop instead of the ISR handler */

    /* Page 26 of documentation: The INTF registers hold which pins caused the interrupt (and only reflect pins that are
    ** enabled for interrupting). We could have also read from the INTCAP registers since they capture the state of the GPIO 
    ** pins when the interrupt occured, but remember that in that scenario, an input has occured if the value is low not high (active-low pushbuttons).
    */
    // int gpio_result_a = mcp->readFromRegister(INTFA);
    // int gpio_result_b = mcp->readFromRegister(INTFB);
    /* This clears the interrupt as well as gets the state of the GPIO ports when the interrupt occured */
    int gpio_result_a = mcp->readFromRegister(INTCAPA);
    int gpio_result_b = mcp->readFromRegister(INTCAPB);

    /* capture the previous state of the push buttons to determine if they are being released */
    bool prev_a_status = push_button_a_status;
    bool prev_b_status = push_button_b_status;
    bool prev_c_status = push_button_c_status;

    biped::Serial(LogLevel::info) << gpio_result_a << "\n" << gpio_result_b << "!\n";
    /* check if button a is being pressed and released */
    if ((gpio_result_a & bit(IOExpanderAPortAPin::pushbutton_a)) == bit(IOExpanderAPortAPin::pushbutton_a)) {
      push_button_a_status = false;
    }
    /* on low the push button is being presseed */
    else {
      push_button_a_status = true;
    }

    /* check if button b is being pressed and released */
    if ((gpio_result_a & bit(IOExpanderAPortAPin::pushbutton_b)) == bit(IOExpanderAPortAPin::pushbutton_b)) {
      push_button_b_status = false;
    }
    /* on low the push button is being presseed */
    else {
      push_button_b_status = true;
    }
    
    /* check if button c is being pressed and released */
    if ((gpio_result_b & bit(IOExpanderAPortBPin::pushbutton_c)) == bit(IOExpanderAPortBPin::pushbutton_c)) {
      push_button_c_status = false;
    }
    /* on low the push button is being presseed */
    else {
      push_button_c_status = true;
    }

    /* only take action if button is being released, not if it was never pressed */
    if (prev_a_status == true && push_button_a_status == false) {
      motor_configuration = (motor_configuration + 1) % 4;
      a_counter++;
    }
    if (prev_b_status == true && push_button_b_status == false) {
      b_counter++;
    }
    if (prev_c_status == true && push_button_c_status == false) {
      c_counter++;
    }
      

    read_i2c = false;
  }
  // TODO set motor direction
  /* update the motor configuration if pushbutton a was pressed */
  
  switch (motor_configuration) {
    case 0:
      /* both motors forward */
      mcp->SetAPin(IOExpanderAPortAPin::motor_left_direction, 1);
      mcp->SetAPin(IOExpanderAPortAPin::motor_right_direction, 1);
      break;
    case 1:
      /* left motor forward, right backwards */
      mcp->SetAPin(IOExpanderAPortAPin::motor_left_direction, 1);
      mcp->SetAPin(IOExpanderAPortAPin::motor_right_direction, 0);
      break;
    case 2:
      /* left motor backwards, right forward */
      mcp->SetAPin(IOExpanderAPortAPin::motor_left_direction, 0);
      mcp->SetAPin(IOExpanderAPortAPin::motor_right_direction, 1);
      break;
    case 3:
      /* both motors backwards */
      mcp->SetAPin(IOExpanderAPortAPin::motor_left_direction, 0);
      mcp->SetAPin(IOExpanderAPortAPin::motor_right_direction, 0);
      break;
  }
  
  Display(0) << "id: " << id;
  Display(1) << "motor configuration: " << motor_configuration;
  Display(2) << "a_counter: "<<a_counter;
  Display(3) << "b_counter: "<<b_counter;
  Display(4) << "c_counter: "<<c_counter;
  Display::display();

  delay(50);
    
}
