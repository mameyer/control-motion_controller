#include "Dispatcher.hpp"
#include <trajectory_follower/Motion2D.hpp>

namespace motion_controller {

double Dispatcher::calcWheelSpeedWhenTurning()
{
    double steeringSpeed = 0.5;
    double wheelSpeed = steeringSpeed * geometry.wheelRadius / geometry.scrubRadius ;
    return wheelSpeed;
}

bool isTurningCenterInside(){
    return true;
}

void Dispatcher::compute(const base::commands::Motion2D& motionCmd, base::samples::Joints &actuatorsCommand, base::samples::Joints &actuatorsFeedback)
{
    status = Idle;
  
    if(motionCmd != base::commands::Motion2D(0., 0., base::Angle::fromRad(0))){
        if (motionCmd.translation == 0 && motionCmd.rotation != 0)
        {
            pointTurn->compute(motionCmd, actuatorsCommand);
            currentMode = ModeTurnOnSpot;
        }
        else if (motionCmd.rotation == 0)
        {
            lateral->compute(motionCmd, actuatorsCommand);
            currentMode = ModeLateral;
        }
        else
        {
            if (!ackermann->compute(motionCmd, actuatorsCommand))
            {
                pointTurn->compute(motionCmd, actuatorsCommand);
                currentMode = ModeTurnOnSpot;
            }
            else
            {
                currentMode = ModeAckermann;
            }
        }
    }

    volatile bool isTooFast = false, needsWaitForTurn = false;
    double maxSpeed = 0;

    for (auto jointActuator: controllerBase->getJointActuators())
    {
        JointCmd* steeringCmd = jointActuator->getJointCmdForType(JointCmdType::Position);
        JointCmd* wheelCmd = jointActuator->getJointCmdForType(JointCmdType::Speed);

        if (!steeringCmd || !wheelCmd)
        {
            continue;
        }

        base::JointState &steeringJS(actuatorsCommand[actuatorsCommand.mapNameToIndex(steeringCmd->getName())]);
        base::JointState &wheelJS(actuatorsCommand[actuatorsCommand.mapNameToIndex(wheelCmd->getName())]);



        if (!base::isUnset<double>(geometry.wheelMaxVel) && geometry.wheelMaxVel > 0)
        {
        
            isTooFast |= std::abs(wheelJS.speed) > geometry.wheelMaxVel;
            
            if (std::abs(wheelJS.speed) > maxSpeed)
            {
                maxSpeed = std::abs(wheelJS.speed);
            }
        }
        
        if(useFeedback){
            base::JointState &steeringJSFb(actuatorsFeedback[actuatorsFeedback.mapNameToIndex(steeringCmd->getName())]);
            needsWaitForTurn |= std::abs(steeringJSFb.position - steeringJS.position) > jointsFeedbackTurningThreshold;
        }
    }

    if (isTooFast)
    {
        status = TooFast;
        double reduce = geometry.wheelMaxVel / maxSpeed;
        for (auto jointActuator: controllerBase->getJointActuators())
        {
            JointCmd* wheelCmd = jointActuator->getJointCmdForType(JointCmdType::Speed);

            if (!wheelCmd)
            {
                continue;
            }

            base::JointState &wheelJS(actuatorsCommand[actuatorsCommand.mapNameToIndex(wheelCmd->getName())]);

            std::cout << "control_motioncontroller: SPEED is over limit! Reducing by " << reduce*100 << "%" << std::endl;
            wheelJS.speed *= reduce;
        }
    }
    
    if(needsWaitForTurn){
        for (auto jointActuator: controllerBase->getJointActuators())
        {
            JointCmd* steeringCmd = jointActuator->getJointCmdForType(JointCmdType::Position);
            JointCmd* wheelCmd = jointActuator->getJointCmdForType(JointCmdType::Speed);
            
            base::JointState &wheelJS(actuatorsCommand[actuatorsCommand.mapNameToIndex(wheelCmd->getName())]);
            base::JointState &steeringJS(actuatorsCommand[actuatorsCommand.mapNameToIndex(steeringCmd->getName())]);
            base::JointState &steeringJSFb(actuatorsFeedback[actuatorsFeedback.mapNameToIndex(steeringCmd->getName())]);
            
            double diff = steeringJS.position - steeringJSFb.position;
            status = NeedsToWaitForTurn;
            if (std::abs(diff) > turningAngleThreshold / 2)
            {
                wheelJS.speed = calcWheelSpeedWhenTurning();
                wheelJS.speed *= diff > 0 ? 1 : -1;
                if(jointActuator->getPosition().y() > 0){
                    //Left side has to turn the opposite way
                    wheelJS.speed *= -1;
                }
            }
            else
            {
                wheelJS.speed = 0;
            }
        }
    }
}

}
