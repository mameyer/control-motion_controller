#pragma once

#include "MotionControllerTypes.hpp"
#include "Ackermann.hpp"
#include "Lateral.hpp"

namespace motion_controller {
    
class Dispatcher
{    
private:
    Geometry geometry;
    ControllerBase *controllerBase;
    bool useFeedback;

    motion_controller::Ackermann *ackermann;
    motion_controller::Lateral *lateral;

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
    }

    ~Dispatcher() {
        delete ackermann;
        delete lateral;
    }

    void setAckermannRatio(double ackermannRatio) {
        ackermann->setAckermannRatio(ackermannRatio);
        lateral->setAckermannRatio(ackermannRatio);
    }

    void compute(const base::commands::Motion2D &motionCmd, base::samples::Joints &actuatorsCommand, base::samples::Joints &actuatorsFeedback);
    inline DriveMode getCurrentMode() { return this->currentMode; };
    inline ControllerStatus getStatus() { return this->status; };
    inline motion_controller::Ackermann *getAckermannController() { return this->ackermann; };
    inline void setTurningAngleThreshold(const double &turningAngleThreshold)
    {
        if (turningAngleThreshold > this->jointsFeedbackTurningThreshold)
        {
            throw std::invalid_argument("wrong argument for turningAngleThreshold");
        }
            
        this->turningAngleThreshold = turningAngleThreshold;
    };
    
    inline void setJointsFeedbackTurningThreshold(const double &jointsFeedbackTurningThreshold)
    {
        if (jointsFeedbackTurningThreshold < this->turningAngleThreshold)
        {
            throw std::invalid_argument("wrong argument for jointsFeedbackTurningThreshold");
        }
        
        this->jointsFeedbackTurningThreshold = jointsFeedbackTurningThreshold;
    };
};

}