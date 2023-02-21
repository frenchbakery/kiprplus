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

#include "aggregation_engine.hpp"

using namespace kp;

#define for_every_motor for (auto & motor : motors)

AggregationEngine::AggregationEngine(AggregationEngine::motorlist_t && m)
: motors(m)
{
}

void AggregationEngine::addMotor(std::shared_ptr<RampedMotor> motor)
{
    motors.push_back(motor);
}

void AggregationEngine::moveRelativePosition(int short speed, int delta_pos)
{
    std::vector<int> starting_positions;
    for_every_motor
    {
        starting_positions.push_back(motor->getPosition());
        motor->moveRelativePosition(speed, delta_pos);
    }

    for_every_motor
    {
        motor->blockMotorDone();
    }
}