/**
 * @file smooth_servo.hpp
 * @author Nilusink
 * @brief implements a better driver for the botball servo
 * @version 0.1
 * @date 2023-03-01
 * 
 * @copyright Copyright FrenchBakery (c) 2023
 * 
 */
#pragma once
#include <thread>
#include <atomic>
#include <kipr/servo/servo.hpp>


namespace kp
{
    // TODO: make a kp::Positionable
    class SmoothServo : kipr::servo::Servo
    {
        protected:
            std::atomic_int start_pos{1024};
            float curr_pos = .0;
            std::atomic_int set_pos{1024};
            std::atomic_int curr_speed{256};  // servo ticks per second

            std::atomic_bool threxit{false};
            std::atomic_bool control_active{false};

            std::thread controller_thread;

            int loop_time = 10;

            void controllerThreadFn();

        public:
            SmoothServo(int port, int initial_pos = 1024);
            ~SmoothServo();

            void disable();
            void enable();
            void setEnabled(bool enabled);
            bool isEnabled() const;

            void setSpeed(int speed);
            int getSpeed();

            void setPosition(int position);
            void setPosition(int position, int speed);
            int getSetPosition();
            int position();

            double getPercentCompleted();
            void waitUntilComleted();
    };
}
