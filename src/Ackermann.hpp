#pragma once

#include "Controller.hpp"

namespace motion_controller {
    
class Ackermann : public Controller
{
private:
    Eigen::Vector2d computeTurningCenter(const trajectory_follower::Motion2D &motionCommand);
    Eigen::Vector2d currentTurningCenter;
    
public:
    Ackermann(const Geometry &geometry, ControllerBase *controllerBase)
        : Controller(geometry, controllerBase)
    {
    }
    
    inline Eigen::Vector2d getTurningCenter() { return currentTurningCenter; };
    inline double getAckermannRation() { return ackermannRatio; };
    virtual const base::samples::Joints& compute(const trajectory_follower::Motion2D &motionCmd);
};
    
}