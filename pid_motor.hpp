/**
 * @file pid_motor.hpp
 * @author melektron
 * @brief Motor with local asynchronous PID position control
 * @version 0.1
 * @date 2023-02-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <atomic>
#include <thread>
#include <kipr/motor/motor.hpp>
#include "sync_pid.hpp"
#include "positionable.hpp"

namespace kp
{

    class PIDMotor : private kipr::motor::Motor, public Positionable
    {
    protected:
        kipr::motor::BackEMF position_provider;
        SyncPID pid_provider;

        // distance from setpoint for the target to be considered reached
        std::atomic_int accuracy{0};
        // flag whether controller thread should do anything
        std::atomic_bool pos_ctrl_active{false};
        // exit flag for thread
        std::atomic_bool threxit{false};

        // target reached event handler
        Positionable::handler_fn_t eh_target_reached = nullptr;
        // flag to specify that the handler should be removed after the first call
        bool callback_only_once = false;

        /**
         * @brief function that will run the positioning in the background
         *
         */
        void controllerThreadFn();
        std::thread controller_thread;

    public:
        PIDMotor(int port, int max_speed = 1500);

        ~PIDMotor();

        /*
        None of the base classes' methods are virtual so
        we cannot specify override and have to override all of
        them to intercept calls.
        */
        void clearPositionCounter();
        void setPidGains(short p, short i, short d, short pd, short id, short dd);
        void pidGains(short &p, short &i, short &d, short &pd, short &id, short &dd);
        void moveAtVelocity(short velocity);
        void moveToPosition(short speed, int goal_pos);
        void moveRelativePosition(short speed, int delta_pos);
        void freeze();
        bool isMotorDone();
        void blockMotorDone();
        void forward();
        void backward();
        void motor(int percent);
        void baasbennaguui(int percent);
        void motorPower(int percent);
        void off();

        // == additional getters and setters

        /**
         * @brief sets the amount of ticks the position is allowed to deviate
         * from the setpoint for the position controller to accept the result and
         * consider the goal reached
         *
         * @param delta allowed ticks delta from position setpoint
         */
        void setAccuracy(int delta) override;

        /**
         * @brief sets the position control target to a specific location
         *
         * @param target_pos target position in motor ticks
         */
        el::retcode setAbsoluteTarget(int target_pos) override;

        /**
         * @brief sets the position control target to a specific location
         * relative to the current value. For example, you might use this
         * to drive by 500 ticks.
         *
         * @param target_distance
         */
        el::retcode setRelativeTarget(int target_distance) override;

        /**
         * @brief enables the PID controller to regulate the motor position.
         * The controller is deactivated as soon as any regular motor function
         * is called.
         *
         */
        el::retcode enablePositionControl() override;

        /**
         * @brief disables position control. This does not clear the current target
         * but rather just stops position control for the moment. It can resumed
         * using enablePositionControl()
         * 
         * @return el::retcode 
         */
        el::retcode disablePositionControl() override;

        // returns the current counter counter (equivalent to using the BackEMF class)
        int getPosition() const override;

        int getTarget() const override;

        /**
         * @return int ticks the motor is away from the target
         */
        int getDistanceFromTarget() const override;

        /**
         * @retval true distance from target is less than specified accuracy
         * @retval false distance from target is greater than specified accuracy
         */
        bool targetReached() const override;

        /**
         * @brief registers an event handler function that is called
         * when a target is reached. Only one handler can be registered,
         * a subsequent registration will overwrite the previous one.
         * 
         * @param handler callback function taking a positionable reference.
         */
        void onTargetReached(Positionable::handler_fn_t handler, bool once = false);
    };

};