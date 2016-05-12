#include "Dispatcher.hpp"
#include "trajectory_follower/Motion2D.hpp"

namespace motion_controller {

double Dispatcher::calcWheelSpeedWhenTurning()
{
    double steeringSpeed = 0.5;
    double wheelSpeed = steeringSpeed * geometry.wheelRadius / geometry.scrubRadius ;
    return wheelSpeed;
}

base::samples::Joints Dispatcher::compute(const trajectory_follower::Motion2D& motionCmd, base::samples::Joints &actuatorsCommand, base::samples::Joints &actuatorsFeedback)
{
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

        if (useFeedback)
        {
            base::JointState &steeringJSFb(actuatorsFeedback[actuatorsFeedback.mapNameToIndex(steeringCmd->getName())]);
            base::JointState &wheelJSFb(actuatorsFeedback[actuatorsFeedback.mapNameToIndex(wheelCmd->getName())]);

            double diff = std::abs(steeringJS.position - steeringJSFb.position);
            if (diff > jointsFeedbackTurningThreshold)
            {
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

        if (!base::isUnset<double>(geometry.wheelMaxVel))
        {
            isTooFast = std::abs(wheelJS.speed) > geometry.wheelMaxVel;
            if (wheelJS.speed > maxSpeed)
            {
                maxSpeed = wheelJS.speed;
            }
        }
    }

    if (isTooFast)
    {
        double reduce = geometry.wheelMaxVel / maxSpeed;

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

            wheelJS.speed *= reduce;
        }
    }

    return actuatorsCommand;
}

}