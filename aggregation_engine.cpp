/**
 * @file aggregate_motor.hpp
 * @author melektron
 * @brief a aggregation engine that can collectively control 
 * multiple motors while ensuring they operate in sync with each other
 * @version 0.1
 * @date 2023-02-21
 * 
 * @copyright Copyright FrenchBakery (c) 2023
 * 
 */

#include <cmath>
#include <iostream>
#include <kipr/time/time.h>

#include "aggregation_engine.hpp"

using namespace kp;

#define UPDATE_PERIOD 2 // ms

AggregationEngine::AggregationEngine(AggregationEngine::motorlist_t && m)
: motors(m)
{
    for (auto &motor : motors)
    {
        movement_modifiers.push_back(1);
    }

    controller_thread = std::thread(&AggregationEngine::controllerThreadFn, this);
}

AggregationEngine::~AggregationEngine()
{
    threxit = true;
    if (controller_thread.joinable())
        controller_thread.join();
}

void AggregationEngine::controllerThreadFn()
{
    while (!threxit)
    {
        msleep(UPDATE_PERIOD);
        {
            std::lock_guard lock(control_lock);
            if (!sequence_running) continue;

            // pause thread when sequence is done
            if (completed_periods >= sequence_periods)
            {
                // approach final position to make sure we really reached it exactly
                for (int m = 0; m < motors.size(); m++)
                {
                    motors[m]->setAbsoluteTarget(starting_positions[m] + position_deltas[m]);
                }
                // pause thread
                sequence_running = false;
                continue;
            }
            
            // move all motors forward by one step
            for (int m = 0; m < motors.size(); m++)
            {
                motors[m]->setAbsoluteTarget(starting_positions[m] + ((position_deltas[m] * completed_periods) / sequence_periods));
            }

            completed_periods++;
        }
    }
}

void AggregationEngine::addMotor(std::shared_ptr<PIDMotor> motor)
{
    abortSequence();
    std::lock_guard lock(control_lock);

    motors.push_back(motor);
    while (movement_modifiers.size() < motors.size())
        movement_modifiers.push_back(1);
}

void AggregationEngine::abortSequence()
{
    std::lock_guard lock(control_lock);
    sequence_running = false;
    starting_positions.clear();
    position_deltas.clear();
    sequence_periods = 0;
    completed_periods = 0;
}

el::retcode AggregationEngine::setMovementModifiers(const std::vector<double> &modifiers)
{
    abortSequence();
    std::lock_guard lock(control_lock);

    if (modifiers.size() != motors.size())
    {
        std::cout << "setMovementModifiers [E]: invalid movement modifier count" << std::endl;
        return el::retcode::err;
    }
    movement_modifiers = modifiers;
    return el::retcode::ok;
}

el::retcode AggregationEngine::moveRelativePosition(int short speed, int delta_pos)
{
    abortSequence();
    std::lock_guard lock(control_lock);

    if (movement_modifiers.size() != motors.size())
    {
        std::cout << "moveRelativePosition [E]: invalid movement modifier count" << std::endl;
        return el::retcode::err;
    }

    for (int m = 0; m < motors.size(); m++)
    {
        starting_positions.push_back(motors[m]->getPosition());
        position_deltas.push_back(delta_pos * movement_modifiers[m]);
    }

    const int ticks_per_period = (UPDATE_PERIOD * speed) / 1000;
    sequence_periods = std::abs(delta_pos) / ticks_per_period;

    // start the thread
    sequence_running = true;

    return el::retcode::ok;
}

el::retcode AggregationEngine::awaitSequenceComplete(int timeout_ms)
{
    if (!sequence_running) return el::retcode::nak;
    uint64_t start_time = systime();
    while (sequence_running)
    {
        msleep(UPDATE_PERIOD);
        if (timeout_ms && (systime() > start_time + timeout_ms))
            return el::retcode::timeout;
    }

    return el::retcode::ok;
}