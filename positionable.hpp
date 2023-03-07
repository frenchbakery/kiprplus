/**
 * @file positionable.h
 * @author melektron
 * @brief interface class for all motor-like objects allowing absolute position control
 * @version 0.1
 * @date 2023-03-06
 * 
 * @copyright Copyright FrenchBakery (c) 2023
 * 
 */

#pragma once

#include <el/retcode.hpp>
#include <functional>

class Positionable
{
public:
    // Methods that all positionalbe objects must implement

    using handler_fn_t = std::function<void(Positionable&)>;

    virtual void setAccuracy(int md) = 0;
    virtual el::retcode setAbsoluteTarget(int tp) = 0;
    virtual el::retcode setRelativeTarget(int td) = 0;
    virtual el::retcode enablePositionControl() = 0;
    virtual el::retcode disablePositionControl() = 0;
    virtual int getTarget() const = 0;
    virtual int getPosition() const = 0;
    virtual int getDistanceFromTarget() const = 0;
    virtual bool targetReached() const = 0;
    virtual void onTargetReached(handler_fn_t handler, bool once = false) = 0;
};