#pragma once

#include "Controller.hpp"

namespace motion_controller {
    
class Ackermann : public Controller
{
private:
    double computeTurningAngle(const Eigen::Vector2d &turningCenter, const Eigen::Vector2d &wheelposition);
    double computeWheelspeed(const Eigen::Vector2d &turningCenter, const Eigen::Vector2d &wheelposition,
                             const double &targetRotation);
    Eigen::Vector2d computeTurningCenter(const trajectory_follower::Motion2D &motionCommand);
    double translateSpeedToRotation(const double &speed);
    
public:
    Ackermann(const Geometry &geometry)
        : Controller(geometry)
    {
    }
    
    virtual const base::samples::Joints& compute(const trajectory_follower::Motion2D &motionCmd);
};
    
}