#pragma once

#include "Controller.hpp"

namespace motion_controller {
    
class Ackermann : public Controller
{
private:
    bool computeTurningAngle(const Eigen::Vector2d &turningCenter, const Eigen::Vector2d &wheelposition, double &turningAngle);
    Eigen::Vector2d computeTurningCenter(const base::commands::Motion2D &motionCommand);
    Eigen::Vector2d currentTurningCenter;
    bool maxRotationAngleReached;
    bool isTurningCenterInside(){
        if(std::abs(currentTurningCenter.y()) > geometry.axleTrack/2){
            if(std::abs(currentTurningCenter.x()) > (geometry.wheelbase + geometry.wheelOffset) /2){
                return true;
            }
        }
        return false;
    }
    
public:
    Ackermann(const Geometry &geometry, ControllerBase *controllerBase)
        : Controller(geometry, controllerBase)
    {
        maxRotationAngleReached = false;
    }
    
    inline Eigen::Vector2d getTurningCenter() { return currentTurningCenter; };
    inline double getAckermannRation() { return ackermannRatio; };
    virtual bool compute(const base::commands::Motion2D &motionCmd, base::samples::Joints& actuatorsCommand);
};
    
}