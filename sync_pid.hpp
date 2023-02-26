/**
 * @file sync_pid.hpp
 * @author melektron
 * @brief Synchronous PID controller class
 * @version 0.1
 * @date 2023-02-25
 *
 * @copyright Copyright FrenchBakery(c) 2023
 *
 * @remark Created using the following video as reference: https://www.youtube.com/watch?v=zOByx3Izf5U
 *
 */

#pragma once

namespace kp
{
    class SyncPID
    {
    protected:
        // PID gains
        double k_p, k_i, k_d;

        // derivative low pass filter parameters
        double tau;

        // controller state
        double integrator;
        double differentiator;
        double previous_error;
        double previous_measurement;

        // setpoint configuration
        double setpoint;
        double setpoint_offset;

        // output limits and output
        double min_output;
        double max_output;
        double positive_deadband;
        double negative_deadband;
        double output;

    public:
        SyncPID();
        /**
         * @brief Construct a new PID controller with gains, limits and filter time set
         * 
         * @param _k_p proportional gain
         * @param _k_i integral gain
         * @param _k_d derivative gain
         * @param _tau derivative filter time
         */
        SyncPID(double _k_p, double _k_i, double _k_d, double _tau, double min, double max);

        /**
         * @brief sets the output deadband value. This is the band of the output value around
         * the zero point that is considered equivalent to zero and is therefore avoided if 
         * position change is requested.
         * 
         * @param pdb deadband in the positive direction
         * @param ndb deadband in the negative direction
         */
        void setOutputDeadband(double pdb, double ndb);

        /**
         * @brief resets the PID controller to the initial state
         * (resets integrator and differentiator memory)
         *
         */
        void reset();

        /**
         * @brief runs all PID calculations to computes the next 
         * output value from the setpoint, the current measurement and
         * the period since the last measurement.
         * 
         * @param period time passed since the last measurement (aka. for how long this measurement is valid, has an effect on I and D portions)
         * @param measurement the current measured value of the target system
         * @return double controller output 
         */
        double update(double period, double measurement);

        /**
         * @returns double The last PID controller output
         */
        double getOutput();

        /**
         * @brief Set the setpoint of the PID controller
         * 
         * @param s setpoint
         */
        void setSetpoint(double s);
    };
};