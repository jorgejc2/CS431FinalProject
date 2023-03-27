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

/*
 *  Use biped namespace.
 */
using namespace biped;

unsigned int id;

/* status of gpio pins so that we can toggle it during next interrupt */
int timer0_gpio_status = 0; 
int timer1_gpio_status = 0;
// TODO Timer ISRs
void timer0_isr (void* arg) {
  /* with registers */
  *(volatile uint32_t *)TIMG_T0CONFIG_REG(1) |= TIMG_T0_ALARM_EN;
  *(volatile uint32_t *)TIMG_INT_CLR_TIMERS_REG(1) |= TIMG_T0_INT_CLR; // should typically be at end of interrupt but this handler is quick enough

  gpio_set_level((gpio_num_t)ESP32Pin::i2c_sda, timer0_gpio_status);
  if (timer0_gpio_status)
    timer0_gpio_status = 0;
  else
    timer0_gpio_status = 1;
  delayMicroseconds(random(1000,2000));
  
  /* with struct */
  // TIMERG1.hw_timer[0].config.alarm_en = 1;
  // TIMERG1.int_clr_timers.t0 = 1;
  
  
};

void timer1_isr (void* arg) {
  /* with registers */
  *(volatile uint32_t *)TIMG_T1CONFIG_REG(1) |= TIMG_T1_ALARM_EN; // re-enable the alarm
  *(volatile uint32_t *)TIMG_INT_CLR_TIMERS_REG(1) |= TIMG_T1_INT_CLR; // clear the interrupt

  gpio_set_level((gpio_num_t)ESP32Pin::i2c_scl, timer1_gpio_status);
  if (timer1_gpio_status)
    timer1_gpio_status = 0;
  else
    timer1_gpio_status = 1;
  delayMicroseconds(random(1000,2000));
  
  /* with struct */
  // TIMERG1.hw_timer[0].config.alarm_en = 1;
  // TIMERG1.int_clr_timers.t0 = 1;
  
  
};

// TODO Function to initialize timers and interrupts

void
setup()
{
    Wire.setPins(ESP32Pin::i2c_sda, ESP32Pin::i2c_scl);
    biped::Serial::setLogLevelMax(SerialParameter::log_level_max);
    Wire.begin();
    EEPROM.begin(EEPROMParameter::size);
    id = static_cast<unsigned int>(EEPROM.read(AddressParameter::eeprom_serial_number));

    biped::Serial::initialize();

    /* set up gpio */
    gpio_set_direction((gpio_num_t)ESP32Pin::i2c_sda, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)ESP32Pin::i2c_scl, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)ESP32Pin::i2c_sda, 0);
    gpio_set_level((gpio_num_t)ESP32Pin::i2c_scl, 0);

    /* initialize isr */
    esp_intr_alloc(ETS_TG1_T0_LEVEL_INTR_SOURCE, 0, timer0_isr, NULL, NULL); 
    esp_intr_alloc(ETS_TG1_T1_LEVEL_INTR_SOURCE, 0, timer1_isr, NULL, NULL); 

    /* with structs */
    // /* configure timer 0 */
    // TIMERG1.hw_timer[0].config.level_int_en = 1;
    // TIMERG1.hw_timer[0].config.divider = 400;
    
    // TIMERG1.hw_timer[0].config.increase = 1; // set counter in up count mode
    // TIMERG1.hw_timer[0].alarm_high = 0; 
    // TIMERG1.hw_timer[0].alarm_low = 1000;
    // TIMERG1.hw_timer[0].load_high = 0;
    // TIMERG1.hw_timer[0].load_low = 0;

    // /* enable alarms and timer */
    // TIMERG1.hw_timer[0].config.alarm_en = 1;
    // TIMERG1.hw_timer[0].config.autoreload = 1; // load specified load value once alarm is fired
    // TIMERG1.hw_timer[0].config.enable = 1;

    /* with registers */

    /* for timer 0 */
    uint32_t mask_1 = TIMG_T0_INCREASE | TIMG_T0_AUTORELOAD | ((TIMG_T0_DIVIDER_V & 400)<<TIMG_T0_DIVIDER_S) | TIMG_T0_LEVEL_INT_EN | TIMG_T0_ALARM_EN;
    *(volatile uint32_t *)TIMG_T0CONFIG_REG(1) |= mask_1;

    *(volatile uint32_t *)TIMG_T0ALARMHI_REG(1) = 0;
    *(volatile uint32_t *)TIMG_T0ALARMLO_REG(1) = 1000;

    *(volatile uint32_t *)TIMG_T0LOADHI_REG(1) = 0;
    *(volatile uint32_t *)TIMG_T0LOADLO_REG(1) = 0;

    /* for timer 1 */
    uint32_t mask_2 = TIMG_T1_INCREASE | TIMG_T1_AUTORELOAD | ((TIMG_T1_DIVIDER_V & 3200)<<TIMG_T1_DIVIDER_S) | TIMG_T1_LEVEL_INT_EN | TIMG_T1_ALARM_EN;
    *(volatile uint32_t *)TIMG_T1CONFIG_REG(1) |= mask_2;

    *(volatile uint32_t *)TIMG_T1ALARMHI_REG(1) = 0;
    *(volatile uint32_t *)TIMG_T1ALARMLO_REG(1) = 1000;

    *(volatile uint32_t *)TIMG_T1LOADHI_REG(1) = 0;
    *(volatile uint32_t *)TIMG_T1LOADLO_REG(1) = 0;

    /* enable both timers */
    *(volatile uint32_t *)TIMG_T0CONFIG_REG(1) |= TIMG_T0_EN;
    *(volatile uint32_t *)TIMG_T1CONFIG_REG(1) |= TIMG_T1_EN;


    // TODO Initialize GPIOs

    biped::Serial(LogLevel::info) << id <<"\nyaohuiw2" << "\nrileycw2" << "\njorgejc2\n";
}

void
loop()
{
    TIMERG1.hw_timer[0].update = 1;
    TIMERG1.hw_timer[1].update = 1;
    biped::Serial(LogLevel::info) << TIMERG1.hw_timer[0].cnt_low << " -- " << TIMERG1.hw_timer[1].cnt_low;
    delay(10);
}

