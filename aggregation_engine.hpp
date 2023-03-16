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
#include <mutex>
#include <thread>
#include <atomic>
#include <el/retcode.hpp>
#include "positionable.hpp"

namespace kp
{
    class AggregationEngine
    {
    protected:
        using motorlist_t = std::vector<std::shared_ptr<Positionable>>;
        motorlist_t motors;

        // lock that makes sure motors don't get added or parameters are not modified while a sequence
        // is running
        std::mutex control_lock;

        // flag telling the controller thread to process a sequence
        bool sequence_running{false};
        // multiplier that will be applied to every motor individually before setting it's position
        std::vector<double> movement_modifiers;
        // positions of all the motors at the beginning of a tracking sequence
        std::vector<int> starting_positions;
        // individual delta values from start to target for each motor in the active sequence with movement modifiers applied
        std::vector<int> position_deltas;
        // number of periods in the active sequence
        int sequence_periods = 0;
        // number of periods already completed in the current sequence
        int completed_periods = 0;

        /**
         * @brief function that will run the positioning in the background
         */
        void controllerThreadFn();
        std::thread controller_thread;
        std::atomic_bool threxit{false};

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
         * @brief Destroy the Aggregation Engine instance and stop controller thread
         */
        ~AggregationEngine();

        /**
         * @brief adds a new motor to the aggregation list
         * 
         * @param motor shared motor to add
         */
        void addMotor(std::shared_ptr<Positionable> motor);

        /**
         * @brief stops the current movement sequence pauses the controller thread
         * and resets all internal values to the defaults.
         */
        void abortSequence();

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

        /**
         * @retval true - sequence currently running
         * @retval false - currently no sequence running, last sequence has been completed
         */
        bool sequenceRunning();

        /**
         * @brief blocks until the current sequence is done processing (this does not take into
         * account whether or not the positionables have actually reached the instructed goal)
         * or the timeout is reached
         * 
         * @param timeout_ms timeout in ms (0 means no timeout)
         * @retval ok - sequence completed
         * @retval timeout - timeout reached before sequence completed
         * @retval nak - no sequence active
         */
        el::retcode awaitSequenceComplete(int timeout_ms = 0);
    };
};
