#include "Dispatcher.hpp"
#include <trajectory_follower/Motion2D.hpp>

namespace motion_controller {

double Dispatcher::calcWheelSpeedWhenTurning()
{
    double steeringSpeed = 0.5;
    double wheelSpeed = steeringSpeed * geometry.wheelRadius / geometry.scrubRadius ;
    return wheelSpeed;
}

void Dispatcher::compute(const trajectory_follower::Motion2D& motionCmd, base::samples::Joints &actuatorsCommand, base::samples::Joints &actuatorsFeedback)
{
    status = Idle;
    
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

    bool isTooFast = false;
    double maxSpeed = std::numeric_limits<double>::min();

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

        if (useFeedback)
        {
            base::JointState &steeringJSFb(actuatorsFeedback[actuatorsFeedback.mapNameToIndex(steeringCmd->getName())]);
            
            double diff = std::abs(steeringJS.position - steeringJSFb.position);
            if (diff > jointsFeedbackTurningThreshold)
            {
                status = NeedsToWaitForTurn;
                if (diff > turningAngleThreshold / 2)
                {
                    wheelJS.speed = calcWheelSpeedWhenTurning();
                    wheelJS.speed *= diff > 0 ? 1 : -1;
                }
                else
                {
                    wheelJS.speed = 0;
                }
            }
        }

        if (!base::isUnset<double>(geometry.wheelMaxVel) && geometry.wheelMaxVel > 0)
        {
//             if (useFeedback)
//             {
//                 base::JointState &wheelJSFb(actuatorsFeedback[actuatorsFeedback.mapNameToIndex(wheelCmd->getName())]);
//                 isTooFast |= std::abs(wheelJSFb.speed) > geometry.wheelMaxVel;   
//             }
//             else
            {
                isTooFast |= std::abs(wheelJS.speed) > geometry.wheelMaxVel;
            }
            
            if (wheelJS.speed > maxSpeed)
            {
                maxSpeed = wheelJS.speed;
            }
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

            std::cout << "control_motioncontroller: SPEED is over limit! Reducing to " << reduce*100 << "%" << std::endl;
            wheelJS.speed *= reduce;
        }
    }
}

}