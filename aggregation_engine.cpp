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
}

void AggregationEngine::addMotor(std::shared_ptr<PIDMotor> motor)
{
    motors.push_back(motor);
}

void AggregationEngine::moveRelativePosition(int short speed, int delta_pos)
{
    const int update_period = 10; // ms

    std::vector<int> starting_positions;
    for (auto &motor : motors)
    {
        starting_positions.push_back(motor->getPosition());
    }
    int ticks_per_period = (update_period * speed) / 1000;
    int periods = std::abs(delta_pos) / ticks_per_period;
    std::cout << ticks_per_period << ", " << periods << std::endl;
    for (int i = 0; i < periods; i++)
    {
        for (auto &motor : motors)
        {
            std::cout << "a" << ticks_per_period * (delta_pos > 0 ? 1 : -1) << "  ";
            motor->setRelativeTarget(ticks_per_period * (delta_pos > 0 ? 1 : -1));
        }
        std::cout << std::endl;
        msleep(update_period);
    }
    // set final target to make sure everything is in the correct location
    for (int i = 0; i < motors.size(); i++)
    {
        std::cout << "b" << starting_positions[i] + delta_pos << "  ";
        motors[i]->setAbsoluteTarget(starting_positions[i] + delta_pos);
    }
    std::cout << std::endl;
}