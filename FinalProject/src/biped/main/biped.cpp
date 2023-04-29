/**
 *  @file   biped.cpp
 *  @author Simon Yu
 *  @date   01/12/2022
 *  @brief  Biped main program source.
 *
 *  This file implements the Biped main program.
 */

/*
 *  External headers.
 */
#include <EEPROM.h>
#include <esp_intr_alloc.h>
#include <ESP32TimerInterrupt.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 *  Project headers.
 */
#include "actuator/actuator.h"
#include "platform/camera.h"
#include "controller/controller.h"
#include "platform/display.h"
#include "common/global.h"
#include "platform/io_expander.h"
#include "task/interrupt.h"
#include "planner/maneuver_planner.h"
#include "common/pin.h"
#include "sensor/sensor.h"
#include "platform/serial.h"
#include "task/task.h"
#include "planner/waypoint_planner.h"

#include "BluetoothSerial.h"
#include <string>

/* Check if Bluetooth configurations are enabled in the SDK */
/* If not, then you have to recompile the SDK */
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

/*
 *  Use biped namespace.
 */
using namespace biped;

/* global variables */
// ActuationCommand * curr_actutation_command;

BluetoothSerial SerialBT;

void moveLeft();
void moveRight();
void moveForward();
void moveBack();
ControllerReference controled_reference;

double pos_x_step_val = 0.05; // meters
double att_z_step_val = 5;   // degrees
/**
 *  @brief  Main program setup function.
 *
 *  This function creates, configures, and launched drivers, objects,
 *  and tasks. The function also sets pin modes and attaches interrupt
 *  handlers to their corresponding pins.
 */
void
setup()
{

    /*
     *  Set pin mode for the I/O expander interrupt pins using
     *  the Arduino pin mode function. Use pull-up if the pin
     *  mode is input.
     *  See the parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    pinMode(ESP32Pin::io_expander_a_interrupt, INPUT_PULLUP);
    pinMode(ESP32Pin::io_expander_b_interrupt, INPUT_PULLUP);

    Serial::initialize();
    biped::Serial::setLogLevelMax(biped::SerialParameter::log_level_max);
    SerialBT.begin("ESP32_Control_Jorge");

    /*
     *  Set I2C driver object (Wire) SDA and SCL pins and set the
     *  serial object maximum log level.
     *  See the parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    Wire.setPins(ESP32Pin::i2c_sda, ESP32Pin::i2c_scl);
    Wire.begin();

    /*
     *  Initialize I2C driver (Wire), EEPROM driver (EEPROM),
     *  display, and serial objects.
     *  See the parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    EEPROM.begin(EEPROMParameter::size);
    Display::initialize();
    

    /*
     *  Instantiate all objects and store their shared pointers.
     *
     *  Note that the order of instantiation matters! The camera
     *  object has to be instantiated first, then the I/O expanders,
     *  and then the rest of the objects.
     *
     *  See the global and parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    // camera_ = std::shared_ptr<Camera>(new Camera());
    gpio_install_isr_service(0);
    io_expander_a_ = std::make_shared<IOExpander>(AddressParameter::io_expander_a);
    io_expander_b_ = std::make_shared<IOExpander>(AddressParameter::io_expander_b);
    actuator_ = std::shared_ptr<Actuator>(new Actuator());
    controller_ = std::shared_ptr<Controller>(new Controller());
    sensor_ = std::shared_ptr<biped::Sensor>(new biped::Sensor());
    /* Change depending on desired Planner*/
    // planner_ = std::shared_ptr<WaypointPlanner>(new WaypointPlanner());
    // planner_ =  std::shared_ptr<ManeuverPlanner>(new ManeuverPlanner());
    timer_ = std::make_shared<ESP32TimerInterrupt>(0);
   
    // controled_reference = std::make_shared<ControllerReference>();

    /*
     *  Read and store the serial number from the EEPROM.
     *  See the global and parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    serial_number_ = static_cast<unsigned>(EEPROM.read(AddressParameter::eeprom_serial_number));

    /*
     *  Set controller periods.
     *  See the controller class for details.
     *
     *  Remember to set both the fast and slow domain
     *  periods, if applicable.
     */
    // TODO LAB 7 YOUR CODE HERE.
    controller_->setPeriod(PeriodParameter::fast , true);
    controller_->setPeriod(PeriodParameter::slow, false);
    // ControllerReference control_ref;
    // control_ref.attitude_y = 0.04;
    // controller_->setControllerReference(control_ref);


    /*
     *  Create I/O expander interrupt tasks using the
     *  FreeRTOS xTaskCreatePinnedToCore function. Set
     *  the task descriptive names to be their task
     *  function names. The tasks have the highest
     *  priority. Pin both tasks to core 1.
     *  See the global and parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    xTaskCreatePinnedToCore(&ioExpanderAInterruptTask , "ioExpanderAInterruptTask", TaskParameter::stack_size, nullptr, TaskParameter::priority_max, &task_handle_io_expander_a_interrupt_, TaskParameter::core_1);
    xTaskCreatePinnedToCore(&ioExpanderBInterruptTask , "ioExpanderBInterruptTask", TaskParameter::stack_size, nullptr, TaskParameter::priority_max, &task_handle_io_expander_b_interrupt_, TaskParameter::core_1);

    /*
     *  Attach the I/O expander and encoder interrupt handlers.
     *  Attach the I/O expander interrupt handlers first in rising mode.
     *  Then, attach the encoder interrupt handlers in change mode.
     *
     *  See the interrupt and parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    biped::attachInterrupt(ESP32Pin::io_expander_a_interrupt, ioExpanderAInterruptHandler, RISING);
    biped::attachInterrupt(ESP32Pin::io_expander_b_interrupt, ioExpanderBInterruptHandler, RISING);
    biped::attachInterrupt(ESP32Pin::motor_left_encoder_a, encoderLeftAInterruptHandler, CHANGE);
    biped::attachInterrupt(ESP32Pin::motor_left_encoder_b, encoderLeftBInterruptHandler, CHANGE);
    biped::attachInterrupt(ESP32Pin::motor_right_encoder_a, encoderRightAInterruptHandler, CHANGE);
    biped::attachInterrupt(ESP32Pin::motor_right_encoder_b, encoderRightBInterruptHandler, CHANGE);

    /*
     *  Set pin mode for the push button pins using
     *  the I/O expander pin mode functions. Use
     *  pull-up if the pin mode is input.
     *  See the parameter header for details.
     */
    // TODO LAB 7 YOUR CODE HERE.
    io_expander_a_->pinModePortA(IOExpanderAPortAPin::pushbutton_a, INPUT_PULLUP);
    io_expander_a_->pinModePortA(IOExpanderAPortAPin::pushbutton_b, INPUT_PULLUP);
    io_expander_a_->pinModePortB(IOExpanderAPortBPin::pushbutton_c, INPUT_PULLUP);

    /*
     *  Attach the push button interrupt handlers using
     *  the I/O expander functions in falling mode.
     *  See the interrupt and parameter header for details.
     */
    // TODO LAB 7 YOUR CODE HERE.
    io_expander_a_->attachInterruptPortA(IOExpanderAPortAPin::pushbutton_b, pushButtonBInterruptHandler, FALLING);
    io_expander_a_->attachInterruptPortA(IOExpanderAPortAPin::pushbutton_a, pushButtonAInterruptHandler, FALLING);
    io_expander_a_->attachInterruptPortB(IOExpanderAPortBPin::pushbutton_c, pushButtonCInterruptHandler, FALLING);


    /*
     *  Create real-time, Wi-Fi, and camera tasks using the
     *  FreeRTOS xTaskCreatePinnedToCore function, in that
     *  order. Set the task descriptive names to be their task
     *  function names. The real-time task has the second
     *  highest priority and the other two tasks have the
     *  lowest priority. Pin all tasks to core 1.
     *  See the global and parameter header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.
    xTaskCreatePinnedToCore(realTimeTask, "realTimeTask", TaskParameter::stack_size, nullptr, TaskParameter::priority_max-1, &task_handle_real_time_, TaskParameter::core_1);
    xTaskCreatePinnedToCore(wiFiTask, "wiFiTask", TaskParameter::stack_size, nullptr, TaskParameter::priority_min, &task_handle_wifi_, TaskParameter::core_1);
    xTaskCreatePinnedToCore(cameraTask, "cameraTask", TaskParameter::stack_size, nullptr, TaskParameter::priority_min, &task_handle_camera_, TaskParameter::core_1);

    /*
     *  Attach the timer interrupt handler to the interrupt timer.
     *  See the interrupt header for details.
     */
    // TODO LAB 6 YOUR CODE HERE.

    timer_->attachInterruptInterval(5000, timerInterruptHandler);

    /*
     *  Print initialization status to serial based on
     *  the current worst log level.
     */
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
 *  @brief  Main program loop task function.
 *
 *  This function is called by the loop task created and launched by
 *  the ESP-IDF framework. The loop task has a low priority and calls
 *  the best-effort task function.
 */
void
loop()
{
    /*
     *  Perform best-effort tasks.
     */
    // TODO LAB 6 YOUR CODE HERE.
    bestEffortTask();

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
    }

    switch(biped::biped_direction_){
        case 1:
        moveLeft();
        break;
        case 2:
        moveForward();
        break;
        case 3:
        moveRight();
        break;
        case 4:
        moveBack();
        break;
    }

    // controller_->setControllerReference(*controled_reference);

    /* DEBUG checking if pushbutton b was pressed */
    // Display(6) << "PB: " << push_button_b_pressed;

    // /* sending actuator commands to motors */
    // ActuationCommand curr_actuation_command(true, true, true, MotorParameter::pwm_max/2, MotorParameter::pwm_max/2);
    // actuator_->actuate(curr_actuation_command);

    /* grab actuation data */
    ActuationCommand curr_command = actuator_->getActuationCommand();

    /* grabbing encoder data */
    biped::EncoderData temp_encoder_data = sensor_->getEncoderData();
    // Display(0) << "id: " << serial_number_;
    Display(4) << "bpd: " << biped::biped_direction_;
    Display(5) << "Pos_x: " << controled_reference.position_x;
    Display(6) << "att_z: " << controled_reference.attitude_z;
    

    // /* grabbing velocity from encoders */
    // EncoderData encoder_data = sensor_->getEncoderData();
    // // Display(0) << "pos_x " << encoder_data.position_x;
    // // Display(1) << "vel_x " << encoder_data.velocity_x;
    // // Display(2) << "steps " << encoder_data.steps;

    /* displaying BMX IMU data */
    biped::IMUData bmx160_imu_data = sensor_->getIMUDataBMX160();

    /* accelartions of BMX */
    // Display(4) << "ax: " << bmx160_imu_data.acceleration_x;
    // Display(5) << "ay: " << bmx160_imu_data.acceleration_y;
    // Display(6) << "az: " << bmx160_imu_data.acceleration_z;
    /* angular velocities of BMX */
    // Display(4) << "avx: " << bmx160_imu_data.angular_velocity_x;
    // Display(5) << "avy: " << bmx160_imu_data.angular_velocity_y;
    // Display(6) << "avz: " << bmx160_imu_data.angular_velocity_z;
    /* compass and heading of BMX */
    // Display(4) << "cx: " << bmx160_imu_data.compass_x;
    // Display(5) << "cy: " << bmx160_imu_data.compass_y;
    // Display(6) << "att_z: " << bmx160_imu_data.attitude_z;

    static float previous_steps_x = 0; // Specifically the position_x from encoder
    static float previous_heading = integrated_angle_x;
    static float previous_position_x = 0;
    static float previous_position_y = 0;

    float dist_x = temp_encoder_data.position_x - previous_steps_x;
    // float new_x = cos(bmx160_imu_data.attitude_z) * dist_x + previous_position_x;
    // float new_y = sin(bmx160_imu_data.attitude_z) * dist_x + previous_position_y;
    float new_x = cos(previous_heading) * dist_x + previous_position_x;
    float new_y = sin(previous_heading) * dist_x + previous_position_y;

    previous_heading = integrated_angle_x;
    previous_steps_x = temp_encoder_data.position_x;
    previous_position_x = new_x; 
    previous_position_y = new_y;

    int new_x_i = int(15 * new_x);
    int new_y_i = int(15 * new_y);
    if (new_x_i > 99)
        new_x_i = 99;
    if (new_x_i < -99)
        new_x_i = -99;
    if (new_y_i > 99)
        new_y_i = 99;
    if (new_y_i < -99)
        new_y_i = -99;
    
    
    char s_y[6] = "Rett\n";
    char s_x[6] = "Lett\n";
    std::string s1 = std::to_string(abs(new_x_i));
    std::string s2 = std::to_string(abs(new_y_i));

    /* string parsing */
    if (new_x_i < 10 && new_x_i >= 0) {
        s_x[1] = ' ';
        s_x[2] = ' ';
        s_x[3] = new_x_i + 48;
    }
    else if (new_x_i > -10 && new_x_i < 0) {
        s_x[1] = ' ';
        s_x[2] = '-';
        s_x[3] = abs(new_x_i) + 48;
    }
    else if (new_x_i >= 10) {
        s_x[1] = ' ';
        for (int i = 0; i < 2; i++)
            s_x[i+2] = s1[i];
    }
    else {
        s_x[1] = '-';
        for (int i = 0; i < 2; i++)
            s_x[i+2] = s1[i];
    }

    if (new_y_i < 10 && new_y_i >= 0) {
        s_y[1] = ' ';
        s_y[2] = ' ';
        s_y[3] = new_y_i + 48;
    }
    else if (new_y_i > -10 && new_y_i < 0) {
        s_y[1] = ' ';
        s_y[2] = '-';
        s_y[3] = abs(new_y_i) + 48;
    }
    else if (new_y_i >= 10) {
        s_y[1] = ' ';
        for (int i = 0; i < 2; i++)
            s_y[i+2] = s2[i];
    }
    else {
        s_y[1] = '-';
        for (int i = 0; i < 2; i++)
            s_y[i+2] = s2[i];
    }

    SerialBT.write((uint8_t*)s_x, 5);
    SerialBT.write((uint8_t*)s_y, 5);
    Display(4) << "new_x: " << s_x;//new_x_i;
    Display(5) << "new_y: " << s_y;//new_y_i;

    /* displaying MPU IMU data */
    biped::IMUData mpu_imu_data = sensor_->getIMUDataMPU6050();
    // biped::TimeOfFlightData tof_data = sensor_->getTimeOfFlightData();
    // // /* Attitude Sensing - IMU READINGS*/
    // Display(7) << "pitch: " << mpu_imu_data.attitude_y;
    // Display(1) << "yaw: " << bmx160_imu_data.attitude_z;
    // Display(2) << "accel_x: " << mpu_imu_data.acceleration_x;
    // Display(3) << "accel_y: " << mpu_imu_data.acceleration_y;
    // Display(4) << "accel_z: " << mpu_imu_data.acceleration_z;
    // Display(5) << "av_x: " << mpu_imu_data.angular_velocity_x;
    // Display(5) << "av_y: " << mpu_imu_data.angular_velocity_y;
    // Display(7) << "av_z: " << mpu_imu_data.angular_velocity_z;
    // // /* Distance Sensing - TOF READINGS*/
    // float tof_left_dist = -1.0;
    // float tof_mid_dist = -1.0;
    // float tof_right_dist = -1.0;
    // tof_left_dist = tof_data.range_left;
    // tof_mid_dist = tof_data.range_middle;
    // tof_right_dist = tof_data.range_right;
    // Display(7) << "tl " << int(tof_left_dist*1000) << " tm " << int(tof_mid_dist*1000) << " tr " << int(tof_right_dist*1000);
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

void moveLeft(){
    double new_head = controled_reference.attitude_z-degreesToRadians(att_z_step_val);
    if (new_head < -M_PI)
        new_head += 2 * M_PI;
    controled_reference.attitude_z = new_head;
    controller_->setControllerReference(controled_reference);
}
void moveRight(){
    double new_head = controled_reference.attitude_z+degreesToRadians(att_z_step_val);
    if (new_head > M_PI)
        new_head = M_PI;
    if (new_head < -M_PI)
        new_head = -M_PI;
    controled_reference.attitude_z = new_head;
    controller_->setControllerReference(controled_reference);
}
void moveForward(){
    controled_reference.position_x+=pos_x_step_val;
    controller_->setControllerReference(controled_reference);
}
void moveBack(){
    controled_reference.position_x-=pos_x_step_val;
    controller_->setControllerReference(controled_reference);
}