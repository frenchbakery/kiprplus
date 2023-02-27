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
#include <el/retcode.hpp>
#include "pid_motor.hpp"

namespace kp
{
    class AggregationEngine
    {
    protected:
        using motorlist_t = std::vector<std::shared_ptr<PIDMotor>>;
        motorlist_t motors;

        // multiplier that will be applied to every motor individually before setting it's position
        std::vector<double> movement_modifiers;
    
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
         * @brief Set a distance multiplier for every motor individually that can be used to invert
         * the motor or counteract accuracy differences.
         * 
         * @param modifiers 
         */
        el::retcode setMovementModifiers(const std::vector<double> &modifiers);

        /**
         * @brief moves all motors by a relative distance 
         * 
         * @param speed average speed to move at (ticks per second)
         * @param delta_pos distance to travel
         */
        el::retcode moveRelativePosition(int short speed, int delta_pos);
    };
};
