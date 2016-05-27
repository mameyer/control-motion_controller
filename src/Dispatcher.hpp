#pragma once

#include "MotionControllerTypes.hpp"
#include <trajectory_follower/Motion2D.hpp>
#include "Ackermann.hpp"
#include "Lateral.hpp"
#include "PointTurn.hpp"

namespace motion_controller {
    
class Dispatcher
{    
private:
    Geometry geometry;
    ControllerBase *controllerBase;
    bool useFeedback;

    motion_controller::Ackermann *ackermann;
    motion_controller::Lateral *lateral;
    motion_controller::PointTurn *pointTurn;

    double turningAngleThreshold;
    double jointsFeedbackTurningThreshold;

    double calcWheelSpeedWhenTurning();
    bool isTurningCenterInside();
    
    DriveMode currentMode;
    ControllerStatus status;

public:
    Dispatcher(const Geometry &geometry, ControllerBase *controllerBase, bool useFeedback)
        : geometry(geometry),
          controllerBase(controllerBase),
          useFeedback(useFeedback)
    {
        turningAngleThreshold = 0.01;
        jointsFeedbackTurningThreshold = 0.1;
        currentMode = Unset;
        status = Idle;
        ackermann = new motion_controller::Ackermann(geometry,controllerBase);
        lateral = new motion_controller::Lateral(geometry,controllerBase);
        pointTurn = new motion_controller::PointTurn(geometry,controllerBase);
    }

    ~Dispatcher() {
        delete ackermann;
        delete lateral;
        delete pointTurn;
    }

    void setAckermannRatio(double ackermannRatio) {
        ackermann->setAckermannRatio(ackermannRatio);
        lateral->setAckermannRatio(ackermannRatio);
        pointTurn->setAckermannRatio(ackermannRatio);
    }

    void compute(const base::commands::Motion2D &motionCmd, base::samples::Joints &actuatorsCommand, base::samples::Joints &actuatorsFeedback);
    inline DriveMode getCurrentMode() { return this->currentMode; };
    inline ControllerStatus getStatus() { return this->status; };
    inline motion_controller::Ackermann *getAckermannController() { return this->ackermann; };
};

}