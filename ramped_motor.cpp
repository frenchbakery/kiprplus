/**
 * @file ramped_motor.hpp
 * @author melektron
 * @brief motor driver class that adds slow-halt and ramping functionality to improve positioning accuracy
 * because the default PID controller does some weird stuff.
 * @version 0.1
 * @date 2023-02-15
 *
 * @copyright Copyright FrenchBakery (c) 2023
 *
 */

#include <cmath>
#include <kipr/time/time.h>

#include "ramped_motor.hpp"
#include "spiaccess.hpp"

using namespace kp;

RampedMotor::RampedMotor(int port)
    : position_provider(port),
      Motor(port)
{
    controller_thread = std::thread(&RampedMotor::controllerThreadFn, this);
}

RampedMotor::~RampedMotor()
{
    threxit = true;
    if (controller_thread.joinable())
        controller_thread.join();
}

void RampedMotor::controllerThreadFn()
{
    while (!threxit)
    {
        if (!pos_ctrl_active)
        {
            msleep(1);
            continue;
        }

        int current_pos = getPosition();
        int delta = std::abs(current_pos - goal_pos);
        distance_traveled = std::abs(current_pos - start_pos);

        if (delta <= max_pos_goal_delta)
        {
            // finished
            freeze();
            pos_ctrl_active = false;
            pos_target_reached = true;
            continue;
        }
        else
        {
            int scaledSpeed = speed;
            // minimum speed that the controller will decelerate to
            // (unless the target speed is lower)
            const int min_speed = 20;
            // the faster the motor is going, the earlier 
            // it has to start decelerating
            const int decel_start = (speed + 500) / 10; // 500 => 100, 1500 => 200
            // the faster the motor is going, the longer it should take to accelerate
            const int accel_end = (speed + 500) / 20;
            if (delta < decel_start)
                scaledSpeed = std::min((int)speed, speed * delta / decel_start + min_speed);
            else if (distance_traveled < accel_end)
            {
                scaledSpeed = std::min((int)speed, speed * distance_traveled / accel_end + min_speed);
            }
            Motor::moveToPosition(scaledSpeed, goal_pos);            
        }

        msleep(50);
    }
}

void RampedMotor::clearPositionCounter()
{
    //auto lock = aquireSPIAccess();
    Motor::clearPositionCounter();
}
void RampedMotor::setPidGains(short p, short i, short d, short pd, short id, short dd)
{
    //auto lock = aquireSPIAccess();
    Motor::setPidGains(p, i, d, pd, id, dd);
}
void RampedMotor::pidGains(short &p, short &i, short &d, short &pd, short &id, short &dd)
{
    //auto lock = aquireSPIAccess();
    Motor::setPidGains(p, i, d, pd, id, dd);
}
void RampedMotor::moveAtVelocity(short velocity)
{
    //auto lock = aquireSPIAccess();
    Motor::moveAtVelocity(velocity);
}
void RampedMotor::moveToPosition(short _speed, int goalPos)
{
    speed = _speed;
    goal_pos = goalPos;
    distance_traveled = 0;
    start_pos = getPosition();
    // if we are already at the target, the control loop will
    // exit immediately
    pos_target_reached = false;
    pos_ctrl_active = true;
}
void RampedMotor::moveRelativePosition(short speed, int deltaPos)
{
    moveToPosition(speed, getPosition() + deltaPos);
}
void RampedMotor::freeze()
{
    pos_ctrl_active = false;
    //auto lock = aquireSPIAccess();
    Motor::freeze();
}
bool RampedMotor::isMotorDone() const
{
    return pos_target_reached;
}
void RampedMotor::blockMotorDone() const
{
    while (!isMotorDone()) msleep(1);
}
void RampedMotor::forward()
{
    pos_ctrl_active = false;
    //auto lock = aquireSPIAccess();
    Motor::forward();
}
void RampedMotor::backward()
{
    pos_ctrl_active = false;
    //auto lock = aquireSPIAccess();
    Motor::backward();
}
void RampedMotor::motor(int percent)
{
    pos_ctrl_active = false;
    //auto lock = aquireSPIAccess();
    Motor::motor(percent);
}
void RampedMotor::baasbennaguui(int percent)
{
    pos_ctrl_active = false;
    //auto lock = aquireSPIAccess();
    Motor::baasbennaguui(percent);
}
void RampedMotor::motorPower(int percent)
{
    pos_ctrl_active = false;
    //auto lock = aquireSPIAccess();
    Motor::motorPower(percent);
}
void RampedMotor::off()
{
    pos_ctrl_active = false;
    //auto lock = aquireSPIAccess();
    Motor::off();
}

void RampedMotor::setAccuracy(int delta)
{
    max_pos_goal_delta = delta;
}

int RampedMotor::getPosition()
{
    //auto lock = aquireSPIAccess();
    return position_provider.value();
}

int RampedMotor::getPercentCompleted()
{
    int full_distance = std::abs(goal_pos - start_pos);
    return full_distance > 0 ? 100 * distance_traveled / full_distance : 100;   // avoid division by 0
}