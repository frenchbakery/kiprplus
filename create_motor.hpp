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

#pragma once

#include <thread>
#include <atomic>
#include <mutex>
#include "positionable.hpp"
#include "sync_pid.hpp"

namespace kp
{
    class CreateMotor : public Positionable
    {
    public:
        static std::mutex create_access_mutex;
    protected:
        static bool create_connected_flag;

        static uint64_t last_position_update;
        
        static SyncPID pid_provider[2];
        static int create_speed[2];
        static int position_offsets[2];
        static int current_position[2];
        static bool pos_ctrl_active[2];
        static int accuracy[2];

        static CreateMotor *instances[2];

        // target reached event handler
        static Positionable::handler_fn_t eh_target_reached[2];
        // flag to specify that the handler should be removed after the first call
        static bool callback_only_once[2];

        // mutex guarding all operations related to starting, or stopping the thread
        static std::mutex mu_thread_ops;
        // number of instances currently needing the thread
        static int thread_consumers;

        // PID controller thread for all instances globally
        static std::thread controller_thread;
        static void controllerThreadFn();

        // create motor "port" of the instance (0 = l, 1 = r)
        int motor_port;

        // reads encoder values and stores them in internal array
        static void updatePositions();

    public:
        static void globalCreateConnect();
        static void globalCreateDisconnect();

        CreateMotor(int p);

        ~CreateMotor();

        /**
         * @brief sets the velocity of the motor
         * 
         * @param v velocity in mm/s from -500 to 500
         * 
         * @retval ok - was set
         * @retval err - invalid velocity
         */
        el::retcode moveAtVelocity(int v);


        void clearPositionCounter();

        void setAccuracy(int md) override;
        el::retcode setAbsoluteTarget(int tp) override;
        el::retcode setRelativeTarget(int td) override;
        el::retcode enablePositionControl() override;
        el::retcode disablePositionControl() override;
        int getTarget() const override;
        int getPosition() const override;
        int getDistanceFromTarget() const override;
        bool targetReached() const override;
        void onTargetReached(Positionable::handler_fn_t handler, bool once = false);
    };
} // namespace kp
