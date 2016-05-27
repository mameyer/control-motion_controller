#pragma once

#include "Controller.hpp"

namespace motion_controller {

class Lateral : public Controller
{
    
public:
    Lateral(const Geometry &geometry, ControllerBase *controllerBase)
        : Controller(geometry, controllerBase)
    {
    }

    virtual bool compute(const base::commands::Motion2D &motionCmd, base::samples::Joints& actuatorsCommand);
};

}
