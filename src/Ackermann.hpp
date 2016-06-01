#pragma once

#include "Controller.hpp"

namespace motion_controller {
    
class Ackermann : public Controller
{
private:
    Eigen::Vector2d computeTurningCenter(const base::commands::Motion2D &motionCommand);
    Eigen::Vector2d currentTurningCenter;
    bool maxRotationAngleReached;
    double rotationMaxEpsilon;
    
public:
    Ackermann(const Geometry &geometry, ControllerBase *controllerBase)
        : Controller(geometry, controllerBase)
    {
        maxRotationAngleReached = false;
        rotationMaxEpsilon = 0.01;
    }
    
    inline Eigen::Vector2d getTurningCenter() { return currentTurningCenter; };
    inline double getAckermannRation() { return ackermannRatio; };
    virtual bool compute(const base::commands::Motion2D &motionCmd, base::samples::Joints& actuatorsCommand);
    inline void setRotationMaxEpsilon(const double &rotationMaxEpsilon) { this->rotationMaxEpsilon = rotationMaxEpsilon; };
};
    
}