#include "Dispatcher.hpp"

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

//actuatorsCommand needs to be configured and set to 0!
void Dispatcher::compute(const base::commands::Motion2D& motionCmd, base::samples::Joints &actuatorsCommand, base::samples::Joints &actuatorsFeedback)
{
    status = Idle;
  
    if(motionCmd != base::commands::Motion2D(0., 0., base::Angle::fromRad(0))){
        status = OK;
        if (motionCmd.rotation != 0)
        {
            ackermann->compute(motionCmd, actuatorsCommand);
            currentMode = ModeAckermann;
        }
        else
        {
            lateral->compute(motionCmd, actuatorsCommand);
            currentMode = ModeLateral;
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
        
        if(std::abs(steeringJS.position) > controllerBase->getMaxRotationAngle()){
            steeringJS.position += M_PI;
            steeringJS.position = fmod(steeringJS.position, M_PI*2);
            wheelJS.speed *= -1;
        }
        
        if (useFeedback)
        {
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
    
    if (needsWaitForTurn)
    {
        for (auto jointActuator: controllerBase->getJointActuators())
        {
            JointCmd* steeringCmd = jointActuator->getJointCmdForType(JointCmdType::Position);
            JointCmd* wheelCmd = jointActuator->getJointCmdForType(JointCmdType::Speed);
            
            base::JointState &wheelJS(actuatorsCommand[actuatorsCommand.mapNameToIndex(wheelCmd->getName())]);
            base::JointState &steeringJS(actuatorsCommand[actuatorsCommand.mapNameToIndex(steeringCmd->getName())]);
            base::JointState &steeringJSFb(actuatorsFeedback[actuatorsFeedback.mapNameToIndex(steeringCmd->getName())]);
            
            double diff = steeringJS.position - steeringJSFb.position;
            status = NeedsToWaitForTurn;
            if (std::abs(diff) > turningAngleThreshold)
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