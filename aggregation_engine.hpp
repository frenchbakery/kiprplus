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

#pragma once

#include <vector>
#include <memory>
#include "pid_motor.hpp"

namespace kp
{
    class AggregationEngine
    {
    protected:
        using motorlist_t = std::vector<std::shared_ptr<PIDMotor>>;
        motorlist_t motors;
    
    public:
        AggregationEngine() = default;
        /**
         * @brief Creates an aggregation engine with 
         * several motors.
         * 
         * @param m list of motors to add
         */
        AggregationEngine(motorlist_t && m);

        /**
         * @brief adds a new motor to the aggregation list
         * 
         * @param motor shared motor to add
         */
        void addMotor(std::shared_ptr<PIDMotor> motor);

        /**
         * @brief moves all motors by a relative distance 
         * 
         * @param speed average speed to move at (ticks per second)
         * @param delta_pos distance to travel
         */
        void moveRelativePosition(int short speed, int delta_pos);         
    };
};
