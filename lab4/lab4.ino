/**
 *  @file   biped.cpp
 *  @author Simon Yu
 *  @date   01/12/2022
 */

/*
 *  External headers.
 */
#include <EEPROM.h>

/*
 *  Project headers.
 */
#include "pin.h"
#include "neopixel.h"
#include "display.h"
#include "serial.h"
#include "parameter.h"
#include "MCP23018.h"

/*
 *  Use biped namespace.
 */
using namespace biped;

unsigned int id;

// TODO create IO expander instance
MCP23018 * mcp; 
/* biped global state variables */
bool push_button_a_status = false;
bool push_button_b_status = false;
bool push_button_c_status = false;
int motor_configuration = 0; // 0: left and right forward, 1: left forward, right backward, 2: left backward, right forward, 3: both backward
int a_counter = 0; // number of times push button a was pressed
int b_counter = 0; // number of times push button b was pressed
int c_counter = 0; // number of times push button c was pressed

// TODO declare synchronization primitives
#define LONG_TIME 0xffff
#define TICKS_TO_WAIT 10
/* this semaphore informs the motorActuator_task to read the i2c and update the state of the biped */
SemaphoreHandle_t read_i2c_sem = nullptr;
/* this semaphore informs the motorActuator_task the buttonPress_task has read the i2c and the motor directions can now be updated depending on the new state of the biped */
SemaphoreHandle_t state_updated_sem = nullptr;
/* this semaphore informs the display_task that the state and motors have been updated, and now we can finally update the display */
SemaphoreHandle_t motors_updated_sem = nullptr;


// TODO define tasks
void buttonPress_task(void * pvParameters) {
  /* this task will be responsible for reading the gpio ports */
  read_i2c_sem = xSemaphoreCreateBinary();
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  while(1) {
    /* block waiting for the read_i2c_sem semaphore to become available */
    if (xSemaphoreTake(read_i2c_sem, LONG_TIME) == pdTRUE) {

      /* reset task awoken flag */
      xHigherPriorityTaskWoken = pdFALSE;

      /* update global state variables by reading the i2c chip */
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

      /* display to the serial monitor what we captured from the i2c chip */
      // biped::Serial(LogLevel::info) << gpio_result_a << "\n" << gpio_result_b << "!\n";
      biped::Serial(LogLevel::info)<<push_button_c_status;

      /* check if button a is being released */
      if ((gpio_result_a & bit(IOExpanderAPortAPin::pushbutton_a)) == bit(IOExpanderAPortAPin::pushbutton_a)) {
        push_button_a_status = false;
      }
      /* on low the push button is being presseed */
      else {
        push_button_a_status = true;
      }

      /* check if button b is being released */
      if ((gpio_result_a & bit(IOExpanderAPortAPin::pushbutton_b)) == bit(IOExpanderAPortAPin::pushbutton_b)) {
        push_button_b_status = false;
      }
      /* on low the push button is being presseed */
      else {
        push_button_b_status = true;
      }
      
      /* check if button c is being released */
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
      /* even though this task is not an ISR, it should still wake up the motorActuator_task */
      xSemaphoreGiveFromISR(state_updated_sem, &xHigherPriorityTaskWoken);

      /* request a context switch if the motorActuator_task was successfully unblocked */
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    }
  }
}

void motorActuator_task(void* pvParameters) {
  /* update the motor directions based on the button logic dictated by the buttonPress_task */
  state_updated_sem = xSemaphoreCreateBinary();
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  while(1) {
    if (xSemaphoreTake(state_updated_sem, LONG_TIME) == pdTRUE) {
      /* reset task awoken flag */
      xHigherPriorityTaskWoken = pdFALSE;
      /* change the motor directions depending on the new state of the biped */
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

      /* even though this task is not an ISR, it should still wake up the display_task */
      xSemaphoreGiveFromISR(motors_updated_sem, &xHigherPriorityTaskWoken);

      /* request a context switch if the motorActuator_task was successfully unblocked */
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}

void display_task(void* pvParameters) {
  /* update the display based on the buttons pressed and the motor configuration dictated by the buttonPress_task */
  motors_updated_sem = xSemaphoreCreateBinary();
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  while(1) {
    if (xSemaphoreTake(motors_updated_sem, LONG_TIME) == pdTRUE) {
      /* update the display depending on the new state of the biped */
      Display(0) << "id: " << id;
      Display(1) << "motor configuration:" << motor_configuration;
      Display(2) << "a_counter: "<<a_counter<<" P: "<<push_button_a_status;
      Display(3) << "b_counter: "<<b_counter<<" P: "<<push_button_b_status;
      Display(4) << "c_counter: "<<c_counter<<" P: "<<push_button_c_status;
      Display::display();
    }
  }
}

/* interrupt service handler */
void gpio_isr () {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  /* give up read_i2c_sem semaphore */
  xSemaphoreGiveFromISR(read_i2c_sem, &xHigherPriorityTaskWoken);
  /* request a context switch if the buttonPress_task task was successfully awoken */
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

  // TODO setup tasks
  /* I decided to create the tasks before setting up the ISR so that the tasks can initialize their semaphores before the ISR tries to use them */
  /* inputs to xTaskCreate: TaskFunction_t pvTaskCode, const char * const pcName, configSTACK_DEPTH_TYPE usStackDepth, void *pvParamters, UBaseType_t uxPriority, TaskHandle_t *pxCreatedTask */
  /*
  ** Notice how the priorities are increasing order. This is because when a task releases a semaphore, it must realease it to a task of higher order. Thus when the i2c ISR yields its semaphore,
  ** we should have a chain effect that goes gpio_isr -> buttonPress_task -> motorActuator_task -> display_task. 
  */
  xTaskCreate( buttonPress_task, "BUTTONPRESS", TaskParameter::stack_size, (void *) 1, TaskParameter::priority_max - 2, nullptr );
  xTaskCreate( motorActuator_task, "MOTORACTUATOR", TaskParameter::stack_size, (void *) 1, TaskParameter::priority_max - 1, nullptr );
  xTaskCreate( display_task, "DISPLAY", TaskParameter::stack_size, (void *) 1, TaskParameter::priority_max, nullptr );
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
  /* do nothing, tasks should be doing everything */
  delay(1000);
}
