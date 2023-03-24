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
#include <fstream>
#include <time.h>
#include "create_motor.hpp"

using namespace kp;

std::mutex CreateMotor::create_access_mutex;

bool CreateMotor::create_connected_flag = false;

uint64_t CreateMotor::last_position_update = 0;

SyncPID CreateMotor::pid_provider[2] = 
{
    SyncPID(1.7, 0, 0, 0, -500, 500),
    SyncPID(1.7, 0, 0, 0, -500, 500)
};
int CreateMotor::create_speed[2] = {0, 0};
int CreateMotor::position_offsets[2] = {0, 0};
int CreateMotor::current_raw_position[2] = {0, 0};
int CreateMotor::current_position[2] = {0, 0};
bool CreateMotor::pos_ctrl_active[2] = {false, false};
int CreateMotor::accuracy[2] = {0, 0};

CreateMotor *CreateMotor::instances[2] = {nullptr, nullptr};

Positionable::handler_fn_t CreateMotor::eh_target_reached[2] = {nullptr, nullptr};
bool CreateMotor::callback_only_once[2] = {false, false};

std::mutex CreateMotor::mu_thread_ops;
int CreateMotor::thread_consumers = 0;

std::thread CreateMotor::controller_thread;

char CreateMotor::file_path[64] = {0};
bool CreateMotor::is_initialized = false;

#define LOOP_DELAY 2 // ms


void CreateMotor::globalCreateConnect()
{
    std::lock_guard lock(create_access_mutex);
    create_connect(); // always returns 0
    set_create_baud_rate(Baud57600);
    create_connected_flag = true;
}

void CreateMotor::globalCreateDisconnect()
{
    std::lock_guard lock(create_access_mutex);
    create_disconnect();
    create_connected_flag = false;
}

// the create should never be communicated with from any thread other than this one
void CreateMotor::controllerThreadFn()
{
    std::time_t t = std::time(nullptr);
    std::strftime(file_path, sizeof(file_path), "/home/access/logs/pid_controller/pid_%m_%Y__%H_%M_%S.log", std::localtime(&t));

    std::cout << "motor log path: " << file_path << std::endl;

    std::ofstream log_stream(file_path);

    char buffer[100];
    short int counter = 0;
    while (thread_consumers)
    {
        counter++;
        updatePositions();
        last_position_update = systime();
        int nr_motors = 2;
        int changed = false;
 
        // log to file
        snprintf(buffer, 99, "%i;%i;%i;%i\n", (int)current_position[0], (int)current_position[1], (int)current_raw_position[0], current_raw_position[1]);

        try
        {
            log_stream << buffer;
        }
        catch (const std::exception &e)
        {
            std::cout << "log write error: " << e.what() << std::endl;
        }

        if (counter > 20)
        {
            counter = 0;
            log_stream.flush();
            is_initialized = true;
        }

 
        for (int i = 0; i < nr_motors; i++)
        {
            if (!pos_ctrl_active[i]) continue;
            changed = true;
            double output = pid_provider[i].update(LOOP_DELAY, current_position[i]);
            create_speed[i] = output;

            /*std::cout << "target=" << std::setw(6) << pid_provider[i].getSetpoint() <<
                         "pos="    << std::setw(6) << current_position[i] <<
                         "offset=" << std::setw(6) << position_offsets[i] <<
                         "output=" <<  std::setw(6) << output << std::endl;*/

            if (eh_target_reached[i] && 
            pid_provider[i].getSetpoint() - current_position[i] < accuracy[i])
            {
                eh_target_reached[i](*(instances[i]));

                if (callback_only_once[i])
                    eh_target_reached[i] = nullptr;
            }
        }
        if (changed)
        {
            std::lock_guard lock(create_access_mutex);
            create_drive_direct(create_speed[0], create_speed[1]);
        }
        
        msleep(LOOP_DELAY);
    }

    log_stream.close();
}

void CreateMotor::updatePositions()
{
    std::lock_guard lock(create_access_mutex);
    short l, r;
    _create_get_raw_encoders(&l, &r);
    //std::cout << std::this_thread::get_id() << " Readpos l=" << l << ", r=" << r << std::endl;

    // filter spikes
    if (is_initialized)
    {
        if (abs(l - current_raw_position[0]) > 500)
        {
            std::cout << "motor 1 spike, resetting!  ";
            int diff = current_raw_position[0] - l ;

            std::cout << "last offset: " << position_offsets[0] << ", difference: " << diff;

            position_offsets[0] -= diff;

            std::cout << ", now: " << position_offsets[0] << std::endl;
        }

        if (abs(r - current_raw_position[1]) > 500)
        {
            std::cout << "motor 2 spike, resetting!  ";
            int diff = current_raw_position[1] - r;
            std::cout << "last offset: " << position_offsets[1] << ", difference: " << diff;

            position_offsets[1] -= diff;

            std::cout << ", now: " << position_offsets[1] << std::endl;
        }
    }

    // set position variables
    current_position[0] = l - position_offsets[0];
    current_position[1] = r - position_offsets[1];

    current_raw_position[0] = l;
    current_raw_position[1] = r;
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
    std::lock_guard lock(create_access_mutex);
    if (v < -500 || v > 500)
        return el::retcode::err;
    disablePositionControl();
    create_speed[motor_port] = v;
    create_drive_direct(create_speed[0], create_speed[1]);
    return el::retcode::ok;
}

void CreateMotor::clearPositionCounter()
{
    // read the raw position from the encoder and save it as the new offset position.
    // This value is then considered zero.
    std::lock_guard lock(create_access_mutex);
    short l, r;
    _create_get_raw_encoders(&l, &r);
    position_offsets[motor_port] = motor_port ? r : l; // motor_port==0 -> l, motor_port==1 -> r
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
    pid_provider[motor_port].reset();
    pos_ctrl_active[motor_port] = true;
    return el::retcode::ok;
}

el::retcode CreateMotor::disablePositionControl()
{
    pos_ctrl_active[motor_port] = false;
    pid_provider[motor_port].reset();
    return el::retcode::ok;
}

int CreateMotor::getTarget() const
{
    return pid_provider[motor_port].getSetpoint();
}

int CreateMotor::getPosition() const
{
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