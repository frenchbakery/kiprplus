/**
 * @file smooth_servo.hpp
 * @author Nilusink
 * @brief implements a better driver for the botball servo
 * @version 0.1
 * @date 2023-03-01
 * 
 * @copyright Copyright FrenchBakery (c) 2023
 * 
 */
#include <cmath>
#include <kipr/time/time.h>
#include "smooth_servo.hpp"
#include <iostream>


using namespace kp;


void SmoothServo::controllerThreadFn()
{
    std::cout << "threat start\n";
    while (!threxit)
    {
        if (control_active)
        {
            if (curr_pos != set_pos)
            {
                float delta = set_pos - curr_pos;
                float max_step = (loop_time / 1000.0) * curr_speed * (delta / std::abs(delta));
                float now_step = (std::abs(delta) > std::abs(max_step)) ? max_step : delta;

                curr_pos += now_step;

                Servo::setPosition((int)curr_pos);
            }
        }
        else
        {
            Servo::setPosition((int)set_pos);
        }
        msleep(loop_time);
    }
    std::cout << "threat exit\n";
}


SmoothServo::SmoothServo(int port, int initial_pos)
    : Servo(port)
{
    if (initial_pos == -1)
    {
        initial_pos = Servo::position();
    }
    start_pos = initial_pos;
    curr_pos = initial_pos;
    set_pos = initial_pos;

    controller_thread = std::thread(&SmoothServo::controllerThreadFn, this);
}


SmoothServo::~SmoothServo()
{
    threxit = true;
    if (controller_thread.joinable())
        controller_thread.join();
}


void SmoothServo::disable()
{
    setEnabled(false);
}

void SmoothServo::enable()
{
    setEnabled(true);
}

void SmoothServo::setEnabled(bool enabled)
{
    control_active = enabled;
    Servo::setEnabled(enabled);
}

bool SmoothServo::isEnabled() const
{
    return Servo::isEnabled();
}


void SmoothServo::setSpeed(int speed)
{
    curr_speed = speed;
}

int SmoothServo::getSpeed()
{
    return curr_speed;
}


void SmoothServo::setPosition(int position)
{
    if (!(0 <= position && position <= 2048))
    {
        std::cout << "SmoothServo: INVALID VALUE FOR POSITION: " << position << std::endl;
        return;
    }
    start_pos = curr_pos;
    set_pos = position;
}

void SmoothServo::setPosition(int position, int speed)
{
    setPosition(position);
    curr_speed = speed;
}

int SmoothServo::getSetPosition()
{
    return set_pos;
}

int SmoothServo::position()
{
    return (int)curr_pos;
}


double SmoothServo::getPercentCompleted()
{
    int delta = set_pos - start_pos;
    if (delta == 0) return 100;

    return ((curr_pos - start_pos) / (delta)) * 100;
}

void SmoothServo::waitUntilComleted()
{
    while (!(getPercentCompleted() > 99)) { msleep(10); };
}

int SmoothServo::getServoPosition()
{
    return Servo::position();
}