/*
 * type.h
 *
 *  Created on: Jan 5, 2023
 *      Author: simonyu
 */

#ifndef COMMON_TYPE_H_
#define COMMON_TYPE_H_

/*
 *  External headers.
 */
#include <limits>
#include <vector>

#include "utility/math.h"

namespace biped
{
/**
 *  @brief  Actuation command struct.
 *
 *  This struct contains actuation command entries,
 *  such as the motor enable, left and right motor directions
 *  and pulse width modulation (PWM) values.
 */
struct ActuationCommand
{
    bool motor_enable;  //!< Motor enable.
    bool motor_left_forward;    //!< Left motor direction.
    bool motor_right_forward;   //!< Right motor direction.
    double motor_left_pwm;   //!< Left motor PWM value.
    double motor_right_pwm;  //!< Right motor PWM value.

    /**
     *  @brief  Actuation command struct constructor.
     *
     *  This constructor initializes all actuation command struct entries.
     */
    ActuationCommand() : motor_enable(false), motor_left_forward(true), motor_right_forward(false),
            motor_left_pwm(0), motor_right_pwm(0)
    {
    }
};

/**
 *  @brief  Controller reference struct.
 *
 *  This struct contains controller reference entries,
 *  such as X position, Y attitude (pitch), and
 *  Z attitude (yaw) references.
 */
struct ControllerReference
{
    double attitude_y;   //!< Y attitude (pitch) controller reference, in radians.
    double attitude_z;   //!< Z attitude (yaw) controller reference, in radians.
    double position_x;   //!< X position controller reference, in meters.

    /**
     *  @brief  Controller reference struct constructor.
     *
     *  This constructor initializes all controller reference struct entries to 0.
     */
    ControllerReference() : attitude_y(degreesToRadians(0)), attitude_z(degreesToRadians(0)),
            position_x(0)
    {
    }
};

/**
 *  @brief  Controller saturation struct.
 *
 *  This struct contains controller saturation entries,
 *  such as the input and output saturations.
 */
struct ControllerSaturation
{
    double input_upper;   //!< Input saturation upper bound.
    double input_lower;   //!< Input saturation lower bound.
    double output_upper;  //!< Output saturation upper bound.
    double output_lower;  //!< Output saturation lower bound.

    /**
     *  @brief  Controller saturation struct constructor.
     *
     *  This constructor initializes all controller saturation
     *  entries to their respective extremes.
     */
    ControllerSaturation() : input_upper(std::numeric_limits<double>::max()),
            input_lower(std::numeric_limits<double>::lowest()),
            output_upper(std::numeric_limits<double>::max()),
            output_lower(std::numeric_limits<double>::lowest())
    {
    }
};

/**
 *  @brief  Encoder data struct.
 *
 *  This struct contains encoder data entries.
 *
 *  All spatial data is in the standard body reference frame.
 *
 *  Standard body reference frame:
 *  https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-refframes
 *
 *  Rotational right-hand rule:
 *  https://en.wikipedia.org/wiki/Right-hand_rule#Rotations
 */
struct EncoderData
{
    double position_x;   //!< In meters.
    double velocity_x;   //!< In meters per second.
    double steps;

    /**
     *  @brief  Encoder data struct constructor.
     *
     *  This constructor initializes all encoder data struct entries to 0.
     */
    EncoderData() : position_x(0), velocity_x(0), steps(0)
    {
    }
};

/**
 *  @brief  IMU data struct.
 *
 *  This struct contains inertial measurement unit (IMU) data entries,
 *  such as attitude, linear velocity, angular velocity,
 *  linear acceleration, and so on.
 *
 *  All spatial data is in the standard body reference frame.
 *
 *  Standard body reference frame:
 *  https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-refframes
 *
 *  Rotational right-hand rule:
 *  https://en.wikipedia.org/wiki/Right-hand_rule#Rotations
 */
struct IMUData
{
    double acceleration_x;   //!< In meters per second squared.
    double acceleration_y;   //!< In meters per second squared.
    double acceleration_z;   //!< In meters per second squared.
    double attitude_x;   //!< Roll, in radians.
    double attitude_y;   //!< Pitch, in radians.
    double attitude_z;   //!< Yaw, in radians.
    double angular_velocity_x;   //!< Roll rate, in radians per second.
    double angular_velocity_y;   //!< Pitch rate, in radians per second.
    double angular_velocity_z;   //!< Yaw rate, in radians per second.
    double compass_x;   //!< In micro Tesla.
    double compass_y;   ///!< In micro Tesla.
    double compass_z;   //!< In micro Tesla.
    double temperature;  //!< In Celsius.

    /**
     *  @brief  IMU data struct constructor.
     *
     *  This constructor initializes all IMU data struct entries to 0.
     */
    IMUData() : acceleration_x(0), acceleration_y(0), acceleration_z(0), attitude_x(0),
            attitude_y(0), attitude_z(0), angular_velocity_x(0), angular_velocity_y(0),
            angular_velocity_z(0), compass_x(0), compass_y(0), compass_z(0), temperature(0)
    {
    }
};

enum class LogLevel
{
    fatal = 0,
    error,
    warn,
    info,
    debug,
    trace
};

/**
 *  @brief  Planner maneuver struct
 *
 *  This struct contains planner maneuver entries,
 *  such as maneuver type, maneuver transition type,
 *  maneuver transition value and the pointer to the
 *  next maneuver. The maneuvers can be chained up in
 *  a linked list fashion. The struct also defines the
 *  planner maneuver type, and maneuver transition type
 *  enum classes.
 */
struct Maneuver
{
    /**
     *  @brief  Planner maneuver type enum class
     *
     *  This enum class defines all possible maneuver types.
     */
    enum class Type
    {
        drive = 0,  //!< Drive forward until the end of this maneuver.
        drive_left, //!< Drive to the left until the end of this maneuver.
        drive_right,    //!< Drive to the right until the end of this maneuver.
        park,   //!< Park or stop until the end of this maneuver.
        reverse,    //!< Reverse until the end of this maneuver.
        reverse_left,   //!< Reverse to the left until the end of this maneuver.
        reverse_right,  //!< Reverse to the right until the end of this maneuver.
    };

    /**
     *  @brief  Planner maneuver transition type enum class
     *
     *  This enum class defines all possible maneuver transition types.
     */
    enum class TransitionType
    {
        duration = 0, //!< Transition to the next maneuver after a certain time duration, in seconds.
        position_x_below, //!< Transition to the next maneuver if the X position is below a certain value, in meters.
        position_x_above //!< Transition to the next maneuver if the X position is above a certain value, in meters.
    };

    TransitionType transition_type; //!< Maneuver transition type.
    double transition_value; //!< Maneuver transition value, the meaning of which depends on the maneuver transition type.
    Type type;  //!< Maneuver type

    /**
     *  @brief  Planner maneuver struct constructor
     *
     *  This constructor initializes all planner maneuver struct entries.
     */
    Maneuver() : transition_type(TransitionType::duration), transition_value(0), type(Type::park)
    {
    }
};

enum class StreamManipulator
{
    carriage_return,
    clear_line,
    endl
};

struct TimeOfFlightData
{
    std::vector<double> ranges_left;   //!< In meters.
    std::vector<double> ranges_middle; //!< In meters.
    std::vector<double> ranges_right;   //!< In meters.
};
}

#endif
