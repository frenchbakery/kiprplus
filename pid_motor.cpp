/**
 * @file pid_motor.hpp
 * @author melektron
 * @brief Motor with local asynchronous PID position control
 * @version 0.1
 * @date 2023-02-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <kipr/time/time.h>
#include <cmath>
#include <iostream>
#include <iomanip>

#include "pid_motor.hpp"

using namespace kp;

#define LOOP_DELAY 1 // ms


PIDMotor::PIDMotor(int port, int max_speed)
    : Motor(port),
    position_provider(port),
    pid_provider(20, 0, 0, 0, -max_speed, max_speed)
{
    controller_thread = std::thread(&PIDMotor::controllerThreadFn, this);
}

PIDMotor::~PIDMotor()
{
    threxit = true;
    if (controller_thread.joinable())
        controller_thread.join();
}

//int c = 0;

void PIDMotor::controllerThreadFn()
{
    while (!threxit)
    {
        if (!pos_ctrl_active)
        {
            msleep(LOOP_DELAY);
            continue;
        }
        int position = getPosition();
        double output = pid_provider.update(LOOP_DELAY, position);
        Motor::moveAtVelocity(output);

        // call event handler for target reached
        if (eh_target_reached && targetReached())
        {
            eh_target_reached(*this);

            if (callback_only_once)
                eh_target_reached = nullptr;
        }
        

        msleep(LOOP_DELAY);
    }
}

void PIDMotor::clearPositionCounter()
{
    Motor::clearPositionCounter();
}
void PIDMotor::setPidGains(short p, short i, short d, short pd, short id, short dd)
{
    Motor::setPidGains(p, i, d, pd, id, dd);
}
void PIDMotor::pidGains(short &p, short &i, short &d, short &pd, short &id, short &dd)
{
    Motor::pidGains(p, i, d, pd, id, dd);
}
void PIDMotor::moveAtVelocity(short velocity)
{
    pos_ctrl_active = false;
    Motor::moveAtVelocity(velocity);
}
void PIDMotor::moveToPosition(short speed, int goal_pos)
{
    pos_ctrl_active = false;
    Motor::moveToPosition(speed, goal_pos);
}
void PIDMotor::moveRelativePosition(short speed, int delta_pos)
{
    pos_ctrl_active = false;
    Motor::moveRelativePosition(speed, delta_pos);
}
void PIDMotor::freeze()
{
    pos_ctrl_active = false;
    Motor::freeze();
}
bool PIDMotor::isMotorDone()
{
    if (pos_ctrl_active)
        return std::abs(pid_provider.getSetpoint() - getPosition()) <= accuracy;
    return Motor::isMotorDone();
}
void PIDMotor::blockMotorDone()
{
    while (!isMotorDone()) msleep(5);
}
void PIDMotor::forward()
{
    pos_ctrl_active = false;
    Motor::forward();
}
void PIDMotor::backward()
{
    pos_ctrl_active = false;
    Motor::backward();
}
void PIDMotor::motor(int percent)
{
    pos_ctrl_active = false;
    Motor::motor(percent);
}
void PIDMotor::baasbennaguui(int percent)
{
    pos_ctrl_active = false;
    Motor::baasbennaguui(percent);
}
void PIDMotor::motorPower(int percent)
{
    pos_ctrl_active = false;
    Motor::motorPower(percent);
}
void PIDMotor::off()
{
    pos_ctrl_active = false;
    Motor::off();
}

void PIDMotor::setAccuracy(int delta)
{
    accuracy = delta;
}

el::retcode PIDMotor::setAbsoluteTarget(int target_pos)
{
    pid_provider.setSetpoint(target_pos);
    return el::retcode::ok;
}

el::retcode PIDMotor::setRelativeTarget(int target_distance)
{
    pid_provider.setSetpoint(pid_provider.getSetpoint() + target_distance);
    return el::retcode::ok;
}

el::retcode PIDMotor::enablePositionControl()
{
    pos_ctrl_active = true;
    return el::retcode::ok;
}

el::retcode PIDMotor::disablePositionControl()
{
    pos_ctrl_active = false;
    return el::retcode::ok;
}

int PIDMotor::getTarget() const
{
    return pid_provider.getSetpoint();
}

int PIDMotor::getPosition() const
{
    return position_provider.value();
}

int PIDMotor::getDistanceFromTarget() const
{
    return pid_provider.getSetpoint() - getPosition();
}

bool PIDMotor::targetReached() const
{
    return getDistanceFromTarget() < accuracy;
}

void PIDMotor::onTargetReached(Positionable::handler_fn_t handler, bool once)
{
    eh_target_reached = handler;
    callback_only_once = once;
}