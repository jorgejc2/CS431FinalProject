/**
 *  @file   biped.cpp
 *  @author Simon Yu
 *  @date   01/12/2022
 */

/*
 *  External headers.
 */
#include <EEPROM.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 *  Project headers.
 */
// #include "actuator/actuator.h"
// #include "controller/controller.h"
#include "platform/display.h"
#include "common/global.h"
#include "platform/io_expander.h"
#include "task/interrupt.h"
#include "common/pin.h"
#include "sensor/sensor.h"
#include "platform/serial.h"
#include "task/task.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "common/parameter.h"
#include "BluetoothSerial.h"

/*
 *  Use biped namespace.
 */
// using namespace biped;

void startCameraServer();

void IRAM_ATTR test() {
    biped::Serial(biped::LogLevel::info) << "test";
}

void IRAM_ATTR button() {
    biped::Serial(biped::LogLevel::info) << "btn";
}

int add_int_handler(const uint8_t pin, void (*f)(void), gpio_int_type_t type) {
    int ret;
    ret = gpio_isr_handler_add((gpio_num_t) pin, (void (*)(void*)) f, nullptr);
    if (ret < 0) {
        return ret;
    }
    ret = gpio_set_intr_type((gpio_num_t) pin, type);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

/* Check if Bluetooth configurations are enabled in the SDK */
/* If not, then you have to recompile the SDK */
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

/* definitions */
#define LONG_TIME 0xffff
#define TICKS_TO_WAIT 10


// Sensor::Sensor * SensorObject;

/* task handlers */
TaskHandle_t xRealTimeTask = NULL;


/* OBSOLETE */
/* semaphores */
// SemaphoreHandle_t timer_sem;
// bool fast_domain = false;
// /* Sensor Task (can be called by either timer)*/
// /* NOTE: for now, we will use the task specified in task.cpp instead of this one since that seems to be the convention
//  * they want us to use for this lab. 
// */
// void sensor_task(void * pvParameters) {
//     timer_sem = xSemaphoreCreateBinary();

//     while (1) {
//         if (xSemaphoreTake(timer_sem, LONG_TIME) == pdTRUE) {

//             /* do sensor stuff */
//             // SensorObject->sense(fast_domain);
//             /* end sensor stuff */
//         }
        
//     }
// }

// High Frequency Timer

void timer0_isr (void* arg) {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    /* clears timer0 interrupt */
    /* with registers */
    *(volatile uint32_t *)TIMG_T0CONFIG_REG(1) |= TIMG_T0_ALARM_EN;
    *(volatile uint32_t *)TIMG_INT_CLR_TIMERS_REG(1) |= TIMG_T0_INT_CLR; // should typically be at end of interrupt but this handler is quick enough

    // xSemaphoreGiveFromISR(timer_sem, &xHigherPriorityTaskWoken);
    vTaskNotifyGiveFromISR(biped::task_handle_real_time_, nullptr);
    // vTaskNotifyGiveIndexedFromISR( task_handle_real_time_, 0, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



// Low Frequency Timer

void timer1_isr (void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    /* clears timer1 interrupt */
    /* with registers */
    *(volatile uint32_t *)TIMG_T1CONFIG_REG(1) |= TIMG_T1_ALARM_EN; // re-enable the alarm
    *(volatile uint32_t *)TIMG_INT_CLR_TIMERS_REG(1) |= TIMG_T1_INT_CLR; // clear the interrupt

    // xSemaphoreGiveFromISR(timer_sem, &xHigherPriorityTaskWoken);
    vTaskNotifyGiveFromISR(biped::task_handle_real_time_, nullptr);
    // vTaskNotifyGiveIndexedFromISR( task_handle_real_time_, 0, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/**
 *  @brief  Arduino setup function.
 *
 *  This function creates, configures, and launches
 *  objects. The function also attaches the interrupt
 *  handlers to the corresponding pins.
 */
void
setup()
{
    Wire.setPins(biped::ESP32Pin::i2c_sda, biped::ESP32Pin::i2c_scl);
    biped::Serial::setLogLevelMax(biped::SerialParameter::log_level_max);
    biped::serial_number_ = static_cast<unsigned>(EEPROM.read(biped::AddressParameter::eeprom_serial_number));
    Wire.begin();
    EEPROM.begin(biped::EEPROMParameter::size);
    biped::Display::initialize();
    biped::Serial::initialize();

    /* setting up for bluetooth communication */
    // Serial.begin(115200);
    SerialBT.begin();
    // Serial.println("Bluetooth started! Ready to par... ");

    /*
     *  Instantiate all objects.
     *  See the defined object pointers at the top.
     *
     *  Note that the order of instantiation matters!
     *  An object must be instantiated first before
     *  its pointer can be used.
     */
    // TODO LAB 5 YOUR CODE HERE.
    biped::io_expander_a_ = std::shared_ptr<biped::IOExpander>(new biped::IOExpander(biped::AddressParameter::io_expander_a));
    biped::io_expander_b_ = std::shared_ptr<biped::IOExpander>(new biped::IOExpander(biped::AddressParameter::io_expander_b));
    /* Learn how to properly initialize NeoPixel later if it's needed */
    // neopixel_ = std::make_shared<NeoPixel>( new NeoPixel() );
    biped::sensor_ = std::shared_ptr<biped::Sensor>(new biped::Sensor());
    // SensorObject = new biped::Sensor::Sensor();
    /*
     *  Set periods of Controller and Sensor.
     *  See the corresponding class for details.
     *
     *  Remember to set both the fast and slow domain
     *  period for each class, if applicable.
     */
    // TODO LAB 5 YOUR CODE HERE.

    /*
     *  Attach the corresponding interrupt handler to the left
     *  motor encoder pin. Make sure to set the interrupt to CHANGE mode
     *  for maximum resolution.
     *
     *  See the Pin enum for details.
     */
    // TODO LAB 6 YOUR CODE HERE.

    /*
     *  Enable timer thus starting the real-time tasks.
     */
    // TODO LAB 5 YOUR CODE HERE.
    /*
        CREATE TASKS
    */
//    xTaskCreate (sensor_task, "SENSORTASK", TaskParameter::stack_size, (void *) 1, TaskParameter::priority_max-2, nullptr);
   xTaskCreate (biped::realTimeTask, "REALTIMETASK", biped::TaskParameter::stack_size, (void *) 1, biped::TaskParameter::priority_max-2, &biped::task_handle_real_time_);

    /*
        SETUP TIMERS
    */
    // esp_intr_alloc(ETS_TG1_T0_LEVEL_INTR_SOURCE, 0, timerInterruptHandler, NULL, NULL); 
    // esp_intr_alloc(ETS_TG1_T1_LEVEL_INTR_SOURCE, 0, timerInterruptHandler, NULL, NULL); 
    esp_intr_alloc(ETS_TG1_T0_LEVEL_INTR_SOURCE, 0, timer0_isr, NULL, NULL); 
    esp_intr_alloc(ETS_TG1_T1_LEVEL_INTR_SOURCE, 0, timer1_isr, NULL, NULL); 

    /* for timer 0 PeriodParameter::fast */
    uint32_t mask_1 = TIMG_T0_INCREASE | TIMG_T0_AUTORELOAD | ((TIMG_T0_DIVIDER_V & 400)<<TIMG_T0_DIVIDER_S) | TIMG_T0_LEVEL_INT_EN | TIMG_T1_ALARM_EN;
    *(volatile uint32_t *)TIMG_T0CONFIG_REG(1) |= mask_1;

    *(volatile uint32_t *)TIMG_T0ALARMHI_REG(1) = 0;
    *(volatile uint32_t *)TIMG_T0ALARMLO_REG(1) = 1000;

    *(volatile uint32_t *)TIMG_T0LOADHI_REG(1) = 0;
    *(volatile uint32_t *)TIMG_T0LOADLO_REG(1) = 0;

    /* for timer 1 PeriodParameter::slow */
    uint32_t mask_2 = TIMG_T1_INCREASE | TIMG_T1_AUTORELOAD | ((TIMG_T1_DIVIDER_V & 3200)<<TIMG_T1_DIVIDER_S) | TIMG_T1_LEVEL_INT_EN | TIMG_T1_ALARM_EN;
    *(volatile uint32_t *)TIMG_T1CONFIG_REG(1) |= mask_2;

    *(volatile uint32_t *)TIMG_T1ALARMHI_REG(1) = 0;
    *(volatile uint32_t *)TIMG_T1ALARMLO_REG(1) = 1000;

    *(volatile uint32_t *)TIMG_T1LOADHI_REG(1) = 0;
    *(volatile uint32_t *)TIMG_T1LOADLO_REG(1) = 0;

    /* enable both timers */
    *(volatile uint32_t *)TIMG_T0CONFIG_REG(1) |= TIMG_T0_EN;
    *(volatile uint32_t *)TIMG_T1CONFIG_REG(1) |= TIMG_T1_EN;

    if (biped::Serial::getLogLevelWorst() <= biped::LogLevel::error)
    {
        biped::Serial(biped::LogLevel::warn) << "Initialized with error(s).";
    }
    else
    {
        biped::Serial(biped::LogLevel::info) << "Initialized.";
    }

    // /* setting up I2C interface for motor control */
    //   /* The class constructor only accepts 3 bits because whoever created the class already knew the I/O 
    // ** expander started at address 0x20. Thus they take three bits and 'or' it with 0x20 to access the 
    // ** correct IO expander. In otherwords, you can either send 0x20/0x27, or simply 0/7.
    // */
    // mcp = new MCP23018(0x20);

    // mcp->begin();

    // /* set MIRROR and INTCC. The rest, espeically BANK and SEQOP, should be cleared */
    // mcp->writeToRegister(IOCON, IOCON_MIRROR | IOCON_INTCC | IOCON_INTPOL);


    // // TODO setup IO expnader interrupts
    // // attachInterrupt(io_expander_a_interrupt, gpio_isr, RISING);

    // // TODO enable motor and set motor speed
    // /* By default, all the pins are set up as outputs. Want to set up push buttons as inputs. Low is the same as setting as output */
    // mcp->SetDirections(BIT(IOExpanderAPortAPin::pushbutton_a) | BIT(IOExpanderAPortAPin::pushbutton_b), BIT(IOExpanderAPortBPin::pushbutton_c));
    // /* set pull up resistor for every pin */
    // mcp->SetPullups(0xFF, 0xFF);
    // /* enable the motor */
    // mcp->SetBPin(IOExpanderAPortBPin::motor_enable, 1);

    // /* interrupt configuration */


    // /* Page 21 of documentation: by using the DEFVAL register as the interrupt on change source, and interrupt occures
    // ** if the pin is the opposite value to its corresponding pin in the DEFVAL register. When the button is not being pressed,
    // ** its GPIO pin will be high. Thus for the pins where the push buttons are connected, we want their values in DEFVAL to be high
    // ** so that an interrupt occurs when the input is low (pushbuttons are active-low).
    // */
    // mcp->writeToRegister(DEFVALA, bit(IOExpanderAPortAPin::pushbutton_a) | bit(IOExpanderAPortAPin::pushbutton_b));
    // mcp->writeToRegister(DEFVALB, bit(IOExpanderAPortBPin::pushbutton_c));

    // /* Page 22 of documentation: set interrupt on change source to be from comparing the input pins to the values in the 
    // ** DEFVAL register, not by their previous state. 
    // */
    // // mcp->writeToRegister(INTCONA, bit(IOExpanderAPortAPin::pushbutton_a) | bit(IOExpanderAPortAPin::pushbutton_b));
    // // mcp->writeToRegister(INTCONB, bit(IOExpanderAPortBPin::pushbutton_c));
    // attachInterrupt(ESP32Pin::io_expander_a_interrupt, gpio_isr, RISING);


    // /* enable interrupt on change from the push buttons */
    // mcp->writeToRegister(INTENA, bit(IOExpanderAPortAPin::pushbutton_a) | bit(IOExpanderAPortAPin::pushbutton_b));
    // mcp->writeToRegister(INTENB, bit(IOExpanderAPortBPin::pushbutton_c));

    // /* set up pin to command left and right motors with pwm signals */
    // pinMode(ESP32Pin::motor_left_pwm, OUTPUT);
    // analogWrite(ESP32Pin::motor_left_pwm, 200);
    // pinMode(ESP32Pin::motor_right_pwm, OUTPUT);
    // analogWrite(ESP32Pin::motor_right_pwm, 200);
}

/**
 *  @brief  Arduino loop function.
 *
 *  This function is periodically called by the
 *  Arduino framework. The function calls the
 *  best-effort task function.
 */
void
loop()
{

    /* loop for bluetooth */

    if (SerialBT.available()) {
        int incoming = SerialBT.read(); //Read what we receive 

        // separate button ID from button value -> button ID is 10, 20, 30, etc, value is 1 or 0
        int button = floor(incoming / 10);
        int value = incoming % 10;

        /* could potentially be simplified to this logic */
        /* In the case a new direction other than our current one is set to true, change biped to that direction */
        if ((button != biped::biped_direction_) && (value == 1))
            biped::biped_direction_ = button;
        else
            biped::biped_direction_ = 0;
        
        // switch (button) {
        // /* left button */
        // case 1:  
        //     if (value) {
        //         biped_direction_ = 1;
        //     }
        //     else {
        //         biped_direction_ = 0;
        //     }
        //     break;
        // /* forward button */
        // case 2:  
        //     if (value) {
        //         biped_direction_ = 2;
        //     }
        //     else {
        //         biped_direction_ = 0;
        //     }
        // /* right button */
        // case 3:  
        //     if (value) {
        //         biped_direction_ = 3;
        //     }
        //     else {
        //         biped_direction_ = 0;
        //     }
        //     break;
        // /* backward button */
        // case 4:
        // if (value) {
        //         biped_direction_ = 4;
        //     }
        //     else {
        //         biped_direction_ = 0;
        //     }
        //     break;
        // }
        
    }

    /* Print the biped direction based on the data received via bluetooth */
    biped::Display(0) << "Biped Direction: " << biped::biped_direction_;

    /*
     *  Perform best-effort tasks.
     */
    // bestEffortTask();

    /* grabbing encoder data */
    // biped::EncoderData temp_encoder_data = sensor_->getEncoderData();
    // Display(0) << "id: " << serial_number_;
    // Display(1) << "Pos_x: " << temp_encoder_data.position_x;
    // Display(2) << "Vel_x: " << temp_encoder_data.velocity_x;
    // Display::display();

    /* grabbing attitude data */
    biped::IMUData bmx160_imu_data = biped::sensor_->getIMUDataMPU6050();
    biped::TimeOfFlightData tof_data = biped::sensor_->getTimeOfFlightData();
    /* Attitude Sensing - IMU READINGS*/
    // biped::Display(0) << "yaw: " << bmx160_imu_data.attitude_z;
    // biped::Display(1) << "accel_x: " << bmx160_imu_data.acceleration_x;
    // biped::Display(2) << "accel_y: " << bmx160_imu_data.acceleration_y;
    // biped::Display(3) << "accel_z: " << bmx160_imu_data.acceleration_z;
    // biped::Display(4) << "av_x: " << bmx160_imu_data.angular_velocity_x;
    // biped::Display(5) << "av_y: " << bmx160_imu_data.angular_velocity_y;
    // biped::Display(6) << "av_z: " << bmx160_imu_data.angular_velocity_z;
    /* Distance Sensing - TOF READINGS*/
    float tof_left_dist = -1.0;
    float tof_mid_dist = -1.0;
    float tof_right_dist = -1.0;
    if (tof_data.ranges_left.size() >= 1) {
        tof_left_dist = tof_data.ranges_left[0];
    }
    if (tof_data.ranges_middle.size() >= 1) {
        tof_mid_dist = tof_data.ranges_middle[0];
    }
    if (tof_data.ranges_right.size() >= 1) {
        tof_right_dist = tof_data.ranges_right[0];
    }
    // biped::Display(7) << "tl " << int(tof_left_dist*1000) << " tm " << int(tof_mid_dist*1000) << " tr " << int(tof_right_dist*1000);
    // Display(0) << "ToF Left: " << tof_left_dist;
    // Display(1) << "ToF Mid: " << tof_mid_dist;
    // Display(2) << "ToF Right: " << tof_right_dist;
    biped::Display::display();
    // biped::Serial(LogLevel::info) << "att_x: " << bmx160_imu_data.attitude_x << " att_y: " << bmx160_imu_data.attitude_y << " att_z: " << bmx160_imu_data.attitude_z
    // << "accel_x: " << bmx160_imu_data.acceleration_x << "accel_y: " << bmx160_imu_data.acceleration_y << "accel_z: " << bmx160_imu_data.acceleration_z 
    // << "av_x: " << bmx160_imu_data.angular_velocity_x << " av_y: " << bmx160_imu_data.angular_velocity_y << " av_z: " << bmx160_imu_data.angular_velocity_z
    // "comp_x: " << bmx160_imu_data.compass_x << " comp_y: " << bmx160_imu_data.compass_y << " comp_z: " << bmx160_imu_data.compass_z;

    // biped::Serial(LogLevel::info) << "EncoderData: " << temp_encoder_data;
    // biped::Serial(LogLevel::info) << "Encoder_x: " << temp_encoder_data.position_x << " Vel_x: " << temp_encoder_data.velocity_x;
    delay(50);

}
