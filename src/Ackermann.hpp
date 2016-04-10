#pragma once

#include "Controller.hpp"

namespace motion_controller {
    
class Ackermann : public Controller
{
private:
    double ackermannRatio;
    double turningCenterX;
    
    Eigen::Vector2d computeTurningCenter(const trajectory_follower::Motion2D &motionCommand);
    Eigen::Vector2d currentTurningCenter;
    
public:
    Ackermann(const Geometry &geometry, ControllerBase *controllerBase)
        : Controller(geometry, controllerBase)
    {
        ackermannRatio = 0.5;
        turningCenterX = 0;
    }
    
    inline Eigen::Vector2d getTurningCenter() { return currentTurningCenter; };
    inline double getAckermannRation() { return ackermannRatio; };
    void setAckermannRatio(double ackermannRatio);
    virtual const base::samples::Joints& compute(const trajectory_follower::Motion2D &motionCmd);
};
    
}