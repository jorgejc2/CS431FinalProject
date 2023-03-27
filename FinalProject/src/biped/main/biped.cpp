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

/*
 *  Use biped namespace.
 */
using namespace biped;

void startCameraServer();

void IRAM_ATTR test() {
    biped::Serial(LogLevel::info) << "test";
}

void IRAM_ATTR button() {
    biped::Serial(LogLevel::info) << "btn";
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
    vTaskNotifyGiveFromISR(task_handle_real_time_, nullptr);
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
    vTaskNotifyGiveFromISR(task_handle_real_time_, nullptr);
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
    Wire.setPins(ESP32Pin::i2c_sda, ESP32Pin::i2c_scl);
    biped::Serial::setLogLevelMax(biped::SerialParameter::log_level_max);
    serial_number_ = static_cast<unsigned>(EEPROM.read(AddressParameter::eeprom_serial_number));
    Wire.begin();
    EEPROM.begin(EEPROMParameter::size);
    Display::initialize();
    Serial::initialize();

    /*
     *  Instantiate all objects.
     *  See the defined object pointers at the top.
     *
     *  Note that the order of instantiation matters!
     *  An object must be instantiated first before
     *  its pointer can be used.
     */
    // TODO LAB 5 YOUR CODE HERE.
    io_expander_a_ = std::shared_ptr<IOExpander>(new IOExpander(AddressParameter::io_expander_a));
    io_expander_b_ = std::shared_ptr<IOExpander>(new IOExpander(AddressParameter::io_expander_b));
    /* Learn how to properly initialize NeoPixel later if it's needed */
    // neopixel_ = std::make_shared<NeoPixel>( new NeoPixel() );
    sensor_ = std::shared_ptr<biped::Sensor>(new biped::Sensor());
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
   xTaskCreate (realTimeTask, "REALTIMETASK", TaskParameter::stack_size, (void *) 1, TaskParameter::priority_max-2, &task_handle_real_time_);

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

    if (biped::Serial::getLogLevelWorst() <= LogLevel::error)
    {
        biped::Serial(LogLevel::warn) << "Initialized with error(s).";
    }
    else
    {
        biped::Serial(LogLevel::info) << "Initialized.";
    }
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
    biped::IMUData bmx160_imu_data = sensor_->getIMUDataMPU6050();
    biped::TimeOfFlightData tof_data = sensor_->getTimeOfFlightData();
    /* Attitude Sensing - IMU READINGS*/
    Display(0) << "yaw: " << bmx160_imu_data.attitude_z;
    Display(1) << "accel_x: " << bmx160_imu_data.acceleration_x;
    Display(2) << "accel_y: " << bmx160_imu_data.acceleration_y;
    Display(3) << "accel_z: " << bmx160_imu_data.acceleration_z;
    Display(4) << "av_x: " << bmx160_imu_data.angular_velocity_x;
    Display(5) << "av_y: " << bmx160_imu_data.angular_velocity_y;
    Display(6) << "av_z: " << bmx160_imu_data.angular_velocity_z;
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
    Display(7) << "tl " << int(tof_left_dist*1000) << " tm " << int(tof_mid_dist*1000) << " tr " << int(tof_right_dist*1000);
    // Display(0) << "ToF Left: " << tof_left_dist;
    // Display(1) << "ToF Mid: " << tof_mid_dist;
    // Display(2) << "ToF Right: " << tof_right_dist;
    Display::display();
    // biped::Serial(LogLevel::info) << "att_x: " << bmx160_imu_data.attitude_x << " att_y: " << bmx160_imu_data.attitude_y << " att_z: " << bmx160_imu_data.attitude_z
    // << "accel_x: " << bmx160_imu_data.acceleration_x << "accel_y: " << bmx160_imu_data.acceleration_y << "accel_z: " << bmx160_imu_data.acceleration_z 
    // << "av_x: " << bmx160_imu_data.angular_velocity_x << " av_y: " << bmx160_imu_data.angular_velocity_y << " av_z: " << bmx160_imu_data.angular_velocity_z
    // "comp_x: " << bmx160_imu_data.compass_x << " comp_y: " << bmx160_imu_data.compass_y << " comp_z: " << bmx160_imu_data.compass_z;

    // biped::Serial(LogLevel::info) << "EncoderData: " << temp_encoder_data;
    // biped::Serial(LogLevel::info) << "Encoder_x: " << temp_encoder_data.position_x << " Vel_x: " << temp_encoder_data.velocity_x;
    delay(50);

}
