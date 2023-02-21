/**
 * @file ramped_motor.hpp
 * @author melektron
 * @brief motor driver class that adds slow-halt and ramping functionality to improve positioning accuracy
 * because the default PID controller does some weird stuff.
 * @version 0.1
 * @date 2023-02-15
 *
 * @copyright Copyright FrenchBakery (c) 2023
 *
 */

#pragma once

#include <thread>
#include <kipr/motors.hpp>

class RampedMotor : public Motor
{
protected:
    BackEMF position_provider;

    // start position of the current position control operation
    int start_pos = 0;
    // current position goal
    int goal_pos = 0;
    // how far the actual position can be from the goal for the position controller to 
    // consider the goal reached
    int max_pos_goal_delta = 0;
    // current set speed
    int speed = 0;
    // flag whether controller thread should do anything
    bool pos_ctrl_active = false;
    // flag set when the positition control target has been reached
    bool pos_target_reached = false;
    // exit flag for thread
    bool threxit = false;

    /**
     * @brief function that will run the positioning in the background
     * 
     */
    void controllerThreadFn();
    std::thread controller_thread;
    

public:
    RampedMotor(int port);
    
    /*
    None of the base classes' methods are virtual so
    we cannot specify override and have to override all of
    them to intercept calls.
    */
    void moveAtVelocity(short velocity);
    void moveToPosition(short speed, int goalPos);
	void moveRelativePosition(short speed, int deltaPos);
    void freeze();
	bool isMotorDone() const;
	void blockMotorDone() const;
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
    void setAccuracy(int delta);

    // returns the current position counter (equivalent to using the BackEMF class)
    int getPosition();



};