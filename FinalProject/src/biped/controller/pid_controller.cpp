/**
 *  @file   pid_controller.cpp
 *  @author Simon Yu
 *  @date   01/13/2022
 *  @brief  PID controller class source.
 *
 *  This file implements the PID controller class.
 */

/*
 *  Project headers.
 */
#include "utility/math.h"
#include "common/parameter.h"
#include "controller/pid_controller.h"
#include "platform/serial.h"
#include <iterator>

/*
 *  Biped namespace.
 */
namespace biped
{
PIDController::PIDController() : state_(0), reference_(0), period_(0), error_differential_(0),
        error_integral_(0)
{
}

double
PIDController::getReference() const
{
    /*
     *  Return the PID controller reference (R).
     */
    // TODO LAB 7 YOUR CODE HERE.
    return reference_;
}

void
PIDController::setGain(const Gain& gain)
{
    /*
     *  Set the PID controller gain.
     */
    // TODO LAB 7 YOUR CODE HERE.

    /*
     *  The integrated error would become meaningless
     *  with new gains. Reset the integrated error
     *  (integral of e).
     */
    // TODO LAB 7 YOUR CODE HERE.
    gain_ = gain;
}

void
PIDController::setSaturation(const ControllerSaturation& saturation)
{
    /*
     *  Set the PID controller saturation.
     */
    // TODO LAB 7 YOUR CODE HERE.
    saturation_ = saturation;
}

void
PIDController::setState(const double& state)
{
    /*
     *  Set the plant state input (Y).
     */
    // TODO LAB 7 YOUR CODE HERE.
    state_ = state;
}

void
PIDController::setReference(const double& reference)
{
    /*
     *  Set the PID controller reference (R).
     */
    // TODO LAB 7 YOUR CODE HERE.
    reference_ = reference;

    /*
     *  The integrated error would become meaningless
     *  with a new reference. Reset the integrated error
     *  (integral of e).
     */
    // TODO LAB 7 YOUR CODE HERE.
    error_integral_ = 0;

}

void
PIDController::setPeriod(const double& period)
{
    /*
     *  Set the PID controller period.
     */
    // TODO LAB 7 YOUR CODE HERE.
    period_ = period;

    /*
     *  The integrated error would become meaningless
     *  with a new control period. Reset the integrated
     *  error (integral of e).
     */
    // TODO LAB 7 YOUR CODE HERE.
    error_integral_ = 0;
}

void
PIDController::setErrorDifferential(const double& error_differential)
{
    /*
     *  Set the error derivative input (delta e).
     */
    // TODO LAB 7 YOUR CODE HERE.
    error_differential_ = error_differential;
}

void
PIDController::resetErrorIntegral()
{
    /*
     *  Reset the integrated error (integral of e) to 0.
     */
    // TODO LAB 7 YOUR CODE HERE.
    error_integral_ = 0;
}

double
PIDController::control()
{
    /*
     *  Validate PID controller period.
     */
    if (period_ <= 0)
    {
        Serial(LogLevel::error) << "Invalid period.";
        return 0;
    }

    /*
     *  Calculate the error (e) between the plant input state (Y)
     *  and PID controller reference (R), and clamp the
     *  calculated error between the input saturation upper
     *  and lower bounds.
     *
     *  For the "correct" signedness, calculate the error (e) by
     *  subtracting state with reference (Y - R). Otherwise, the signs
     *  of your PID controller gains may be flipped.
     *
     *  See Lecture 13 for the definition of
     *  the PID controller.
     */
    // TODO LAB 7 YOUR CODE HERE.
    /* DEBUG should be reference minus state, but TA set up incorrectly */
    double error = state_ - reference_;
    // double error = reference_ - state_;

    /* saturating the input */
    if (error > saturation_.input_upper)
        error = saturation_.input_upper;
    if (error < saturation_.input_lower)
        error = saturation_.input_lower;

    /*
     *  Calculate the new discrete integral of error (integral of e),
     *  and clamp the calculated integral of error (integral of e)
     *  between the negative and positive maximum integrated error
     *  from the PID controller gain struct.
     *
     *  The new discrete integrated error is the current integrated
     *  error plus the product between the PID controller period
     *  and the current error (e).
     *
     *  The above is essentially the Riemann sum:
     *  https://en.wikipedia.org/wiki/Riemann_sum
     *
     *  See Lecture 13 for the definition of
     *  the PID controller.
     */
    // TODO LAB 7 YOUR CODE HERE.
    // error_integral_ += error * period_;
    // if (error_integral_ > gain_.integral_max)
    //     error_integral_ = gain_.integral_max;
    // if (error_integral_ < -gain_.integral_max)
    //     error_integral_ = -gain_.integral_max;

    /*
     *  Calculate the proportional output.
     *
     *  See Lecture 13 for the definition of
     *  the PID controller.
     */
    // TODO LAB 7 YOUR CODE HERE.
    double proportional_output = gain_.proportional * error;

    /*
     *  Calculate the integral output.
     *
     *  See Lecture 13 for the definition of
     *  the PID controller.
     */
    // TODO LAB 7 YOUR CODE HERE.
    double integral_output = gain_.integral * error_integral_;

    /*
     *  Calculate the differential output.
     *
     *  In theory, the differential output is proportional
     *  to the time derivative of the error (e). Instead of
     *  calculating the derivative of the error (e), we
     *  measure the derivative for better performance. For
     *  example, for position control, the differential
     *  output would be proportional to the derivative of
     *  the error (e), which, in this example, is simply
     *  velocity (assuming the target is unchanged over
     *  time.) Instead of calculating the velocity, we
     *  measure it using the sensors (encoders). Here,
     *  the measured error derivative is stored in the
     *  error differential member variable.
     *
     *  See Lecture 13 for the definition of
     *  the PID controller.
     */
    // TODO LAB 7 YOUR CODE HERE.
    double derivative_output = gain_.differential * error_differential_;

    /*
     *  Sum up all the above outputs.
     */
    // TODO LAB 7 YOUR CODE HERE.
    double control_output = proportional_output + integral_output + derivative_output;

    /*
     *  Return the sum of the outputs clamped between the output
     *  saturation upper and lower bounds as the final output
     *  of the PID controller.
     */
    // TODO LAB 7 YOUR CODE HERE.
    if (control_output > saturation_.output_upper)
        control_output = saturation_.output_upper;
    if (control_output < saturation_.output_lower)
        control_output = saturation_.output_lower;

    return control_output;

}
}   // namespace biped
