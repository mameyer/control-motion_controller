#pragma once

#include "MotionControllerTypes.hpp"
#include <trajectory_follower/Motion2D.hpp>
#include "Ackermann.hpp"
#include "Lateral.hpp"
#include "PointTurn.hpp"


namespace motion_controller {
    
    class Dispatcher{
    private:
        Geometry geometry;
        ControllerBase *controllerBase;
        bool useFeedback = false;
        
        motion_controller::Ackermann *ackermann;
        motion_controller::Lateral *lateral;
        motion_controller::PointTurn *pointTurn;
        
        double magicAckermannThreshold = 0.01;
        double magicJointsFeedbackTurningThreshold = 0.1;
        
        double calcWheelSpeedWhenTurning();
        
    public:
        Dispatcher(const Geometry &geometry, ControllerBase *controllerBase, bool useFeedback)
        : geometry(geometry),
          controllerBase(controllerBase),
          useFeedback(useFeedback)
        {
            ackermann = new motion_controller::Ackermann(geometry,controllerBase);
            lateral = new motion_controller::Lateral(geometry,controllerBase);
            pointTurn = new motion_controller::PointTurn(geometry,controllerBase);
        }
        
        ~Dispatcher(){
            delete ackermann;
            delete lateral;
            delete pointTurn;
        }
        
        void setAckermannRatio(double ackermannRatio){
            ackermann->setAckermannRatio(ackermannRatio);
            lateral->setAckermannRatio(ackermannRatio);
            pointTurn->setAckermannRatio(ackermannRatio);
        }
        
        base::samples::Joints compute(const trajectory_follower::Motion2D &motionCmd, base::samples::Joints &actuatorsCommand, base::samples::Joints &actuatorsFeedback);
        
    };
    
    
    
    
}