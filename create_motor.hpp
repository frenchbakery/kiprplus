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

#include "positionable.hpp"

namespace kp
{
    class CreateMotor : public Positionable
    {
    protected:
        static bool create_connected_flag;

    public:
        CreateMotor(int port);


        void setAccuracy(int md) override;
        el::retcode setAbsoluteTarget(int tp) override;
        el::retcode setRelativeTarget(int td) override;
        el::retcode enablePositionControl() override;
        el::retcode disablePositionControl() override;
        int getTarget() const override;
        int getPosition() const override;
        int getDistanceFromTarget() const override;
        bool targetReached() const override;
    };
} // namespace kp
