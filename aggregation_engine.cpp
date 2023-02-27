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

#define for_every_motor for (auto & motor : motors)

AggregationEngine::AggregationEngine(AggregationEngine::motorlist_t && m)
: motors(m)
{
    for (auto &motor : motors)
    {
        movement_modifiers.push_back(1);
    }
}

void AggregationEngine::addMotor(std::shared_ptr<PIDMotor> motor)
{
    motors.push_back(motor);
    while (movement_modifiers.size() < motors.size())
        movement_modifiers.push_back(1);
}

el::retcode AggregationEngine::setMovementModifiers(const std::vector<double> &modifiers)
{
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
    if (movement_modifiers.size() != motors.size())
    {
        std::cout << "moveRelativePosition [E]: invalid movement modifier count" << std::endl;
        return el::retcode::err;
    }
    const int update_period = 10; // ms

    std::vector<int> starting_positions;
    for (auto &motor : motors)
    {
        starting_positions.push_back(motor->getPosition());
    }
    int ticks_per_period = (update_period * speed) / 1000;
    int periods = std::abs(delta_pos) / ticks_per_period;
    for (int i = 0; i < periods; i++)
    {
        for (int i = 0; i < motors.size(); i++)
        {
            motors[i]->setRelativeTarget(ticks_per_period * (delta_pos > 0 ? 1 : -1) * movement_modifiers[i]);
        }
        msleep(update_period);
    }
    // set final target to make sure everything is in the correct location
    for (int i = 0; i < motors.size(); i++)
    {
        motors[i]->setAbsoluteTarget(starting_positions[i] + delta_pos);
    }

    return el::retcode::ok;
}