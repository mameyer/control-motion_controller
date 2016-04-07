#pragma once

#include "Controller.hpp"

namespace motion_controller {

class Lateral : public Controller
{
private:
    double translateSpeedToRotation(double speed);
    
public:
    Lateral(const Geometry &geometry, ControllerBase *controllerBase)
        : Controller(geometry, controllerBase)
    {
    }

    virtual const base::samples::Joints& compute(const trajectory_follower::Motion2D &motionCmd);
};

}
