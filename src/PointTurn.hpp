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

    virtual bool compute(const trajectory_follower::Motion2D &motionCmd, base::samples::Joints& actuatorsCommand);
};
    
}