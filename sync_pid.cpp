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

#include <algorithm>
#include <stdio.h>

#include "sync_pid.hpp"

/**
 * @brief maps a number from an input range to an output range
 * 
 * @param x input number
 * @param imin input range minimum
 * @param imax input range maximum
 * @param omin output range minimum
 * @param omax output range maximum
 * @return double output value
 */
double dmap(double x, double imin, double imax, double omin, double omax)
{
    return (x - imin) * (omax - omin) / (imax - imin) + omin;
}

namespace kp
{

    SyncPID::SyncPID()
    {
        reset();
    }

    SyncPID::SyncPID(double _k_p, double _k_i, double _k_d, double _tau, double min, double max)
        : k_p(_k_p), k_i(_k_i), k_d(_k_d), tau(_tau), min_output(min), max_output(max)
    {
        reset();
    }

    void SyncPID::setOutputDeadband(double pdb, double ndb)
    {
        positive_deadband = pdb;
        negative_deadband = ndb;
    }

    void SyncPID::reset()
    {
        integrator = 0;
        differentiator = 0;
        previous_error = 0;
        previous_measurement = 0;
        setpoint = 0;
        setpoint_offset = 0;
        positive_deadband = 0;
        negative_deadband = 0;
    }

    double SyncPID::update(double period, double measurement)
    {
        double error = setpoint - measurement;

        //printf("p=%lf, i=%lf, d=%lf, period=%lf, meas=%lf, integ=%lf, differ=%lf\n", k_p, k_i, k_d, period, measurement, integrator, differentiator);

        // Proportional
        double proportional = k_p * error;

        // Integral
        integrator = integrator + 0.5 * k_i * period * (error + previous_error);

        //printf("integrator=%lf", integrator);

        // integrator limits
        double min_integ, max_integ;
        if (max_output > proportional)
            max_integ = max_output - proportional;
        else
            max_integ = 0;

        if (min_output < proportional)
            min_integ = min_output - proportional;
        else
            min_integ = 0;

        if (integrator > max_integ)
            integrator = max_integ;
        else if (integrator < min_integ)
            integrator = min_integ;

        // Derivative
        differentiator = (2 * k_d * (measurement - previous_measurement) + (2 * tau - period) * differentiator) / (2 * tau + period);

        // Combine output
        output = proportional + integrator + differentiator;
        //printf("p=%lf, i=%lf, d=%lf, out=%lf", proportional, integrator, differentiator, output);
        
        output = std::min(output, max_output);
        output = std::max(output, min_output);

        // compute deadband
        if (output > 0)
            output = dmap(output, 0, max_output, positive_deadband, max_output);
        else if (output < 0)
            output = dmap(output, 0, min_output, -negative_deadband, min_output);

        // store values for next update
        previous_error = error;
        previous_measurement = measurement;

        //printf("out=%lf", output);

        return output;
    }

    double SyncPID::getOutput()
    {
        return output;
    }

    void SyncPID::setSetpoint(double s)
    {
        setpoint = s;
    }

    double SyncPID::getSetpoint() const
    {
        return setpoint;
    }
};