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
    const int update_period = 2; // ms

    std::vector<int> starting_positions;
    std::vector<int> deltas;
    std::cout << "Main: " << delta_pos << std::endl;
    for (int m = 0; m < motors.size(); m++)
    {
        starting_positions.push_back(motors[m]->getPosition());
        deltas.push_back(delta_pos * movement_modifiers[m]);
        std::cout << "Motor " << m << ": " << deltas.back() << std::endl;
    }
    int ticks_per_period = (update_period * speed) / 1000;
    int periods = std::abs(delta_pos) / ticks_per_period;
    for (int i = 0; i < periods; i++)
    {
        for (int m = 0; m < motors.size(); m++)
        {
            motors[m]->setAbsoluteTarget(starting_positions[m] + ((deltas[m] * i) / periods));
        }
        msleep(update_period);
    }
    // set final target to make sure everything is in the correct location
    for (int m = 0; m < motors.size(); m++)
    {
        motors[m]->setAbsoluteTarget(starting_positions[m] + deltas[m]);
    }

    return el::retcode::ok;
}