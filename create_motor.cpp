/**
 * @file create_motor.hpp
 * @author melektron
 * @brief positionable implementation for the create's motor that uses a PID controller for positioning
 * @version 0.1
 * @date 2023-03-06
 *
 * @copyright Copyright FrenchBakery (c) 2023
 *
 */

#include <kipr/create/create.h>
#include <kipr/time/time.h>
#include <iostream>
#include <iomanip>
#include "create_motor.hpp"

using namespace kp;

bool CreateMotor::create_connected_flag = false;

SyncPID CreateMotor::pid_provider[2] = 
{
    SyncPID(3, 0, 0, 0, -500, 500),
    SyncPID(3, 0, 0, 0, -500, 500)
};
int CreateMotor::create_speed[2] = {0, 0};
int CreateMotor::position_offsets[2] = {0, 0};
int CreateMotor::current_position[2] = {0, 0};
bool CreateMotor::pos_ctrl_active[2] = {false, false};
int CreateMotor::accuracy[2] = {0, 0};

CreateMotor *CreateMotor::instances[2] = {nullptr, nullptr};

Positionable::handler_fn_t CreateMotor::eh_target_reached[2] = {nullptr, nullptr};
bool CreateMotor::callback_only_once[2] = {false, false};

std::mutex CreateMotor::mu_thread_ops;
int CreateMotor::thread_consumers = 0;

std::thread CreateMotor::controller_thread;



#define LOOP_DELAY 10 // ms


void CreateMotor::globalCreateConnect()
{
    create_connect(); // always returns 0
    create_connected_flag = true;
}

void CreateMotor::globalCreateDisconnect()
{
    create_disconnect();
    create_connected_flag = false;
}

void CreateMotor::controllerThreadFn()
{
    while (thread_consumers)
    {
        int nr_motors = 2;
        int changed = false;
        for (int i = 0; i < nr_motors; i++)
        {
            if (!pos_ctrl_active[i]) continue;
            changed = true;
            updatePositions();
            double output = pid_provider[i].update(LOOP_DELAY, current_position[i]);
            create_speed[i] = output;

            std::cout << "target=" << std::setw(6) << pid_provider[i].getSetpoint() <<
                         "pos="    << std::setw(6) << current_position[i] <<
                         "offset=" << std::setw(6) << position_offsets[i] <<
                         "output=" <<  std::setw(6) << output << std::endl;

            if (eh_target_reached[i] && 
            pid_provider[i].getSetpoint() - current_position[i] < accuracy[i])
            {
                eh_target_reached[i](*(instances[i]));

                if (callback_only_once[i])
                    eh_target_reached[i] = nullptr;
            }
        }
        if (changed)
            create_drive_direct(create_speed[0], create_speed[1]);
        
        msleep(LOOP_DELAY);
    }
}

void CreateMotor::updatePositions()
{
    short l, r;
    _create_get_raw_encoders(&l, &r);
    std::cout << "Readpos l=" << l << ", r=" << r << std::endl;
    current_position[0] = l - position_offsets[0];
    current_position[1] = r - position_offsets[1];
}


CreateMotor::CreateMotor(int p)
    : motor_port(p)
{
    if (motor_port > 1) std::cout << "ERROR: invalid create motor port, WILL could cause memory corruption!" << std::endl;
    instances[motor_port] = this;

    if (!create_connected_flag)
        globalCreateConnect();
    
    std::lock_guard lock(mu_thread_ops);
    if (!thread_consumers)
        controller_thread = std::thread(CreateMotor::controllerThreadFn);
    thread_consumers++;
}

CreateMotor::~CreateMotor()
{
    std::lock_guard lock(mu_thread_ops);
    thread_consumers--;
    if (!thread_consumers && controller_thread.joinable())
        controller_thread.join();
}

el::retcode CreateMotor::moveAtVelocity(int v)
{
    if (v < -500 || v > 500)
        return el::retcode::err;
    disablePositionControl();
    create_speed[motor_port] = v;
    create_drive_direct(create_speed[0], create_speed[1]);
    return el::retcode::ok;
}

void CreateMotor::clearPositionCounter()
{
    updatePositions();
    position_offsets[motor_port] = current_position[motor_port];
}

void CreateMotor::setAccuracy(int delta)
{
    accuracy[motor_port] = delta;
}

el::retcode CreateMotor::setAbsoluteTarget(int target_pos)
{
    pid_provider[motor_port].setSetpoint(target_pos);
    return el::retcode::ok;
}

el::retcode CreateMotor::setRelativeTarget(int target_distance)
{
    pid_provider[motor_port].setSetpoint(pid_provider[motor_port].getSetpoint() + target_distance);
    return el::retcode::ok;
}

el::retcode CreateMotor::enablePositionControl()
{
    pos_ctrl_active[motor_port] = true;
    return el::retcode::ok;
}

el::retcode CreateMotor::disablePositionControl()
{
    pos_ctrl_active[motor_port] = false;
    return el::retcode::ok;
}

int CreateMotor::getTarget() const
{
    return pid_provider[motor_port].getSetpoint();
}

int CreateMotor::getPosition() const
{
    updatePositions();
    return current_position[motor_port];
}

int CreateMotor::getDistanceFromTarget() const
{
    return pid_provider[motor_port].getSetpoint() - getPosition();
}

bool CreateMotor::targetReached() const
{
    return getDistanceFromTarget() < accuracy[motor_port];
}

void CreateMotor::onTargetReached(Positionable::handler_fn_t handler, bool once)
{
    eh_target_reached[motor_port] = handler;
    callback_only_once[motor_port] = once;
}