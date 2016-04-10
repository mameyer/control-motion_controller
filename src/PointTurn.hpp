#pragma once

#include "Controller.hpp"

namespace motion_controller
{
    
class PointTurn : public Controller
{
public:
    PointTurn(const Geometry &geometry, ControllerBase *controllerBase)
        : Controller(geometry, controllerBase)
    {
    }

    virtual const base::samples::Joints& compute(const trajectory_follower::Motion2D &motionCmd);
};
    
}