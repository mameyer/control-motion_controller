#include "Ackermann.hpp"

namespace motion_controller {

Eigen::Vector2d Ackermann::computeTurningCenter(const base::commands::Motion2D& motionCommand)
{
    double radius = std::abs(motionCommand.translation)/motionCommand.rotation;
    return Eigen::Vector2d(turningCenterX, radius);
}

bool Ackermann::compute(const base::commands::Motion2D& motionCmd, base::samples::Joints& actuatorsCommand)
{   
    base::commands::Motion2D motionCmd_u = motionCmd;
    currentTurningCenter = computeTurningCenter(motionCmd_u);
    if (isTurningCenterInside())
    {
        motionCmd_u.translation = 0;
        currentTurningCenter = computeTurningCenter(motionCmd_u);
    }

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

        if (motionCmd_u.rotation > 0.01 || motionCmd_u.rotation < -0.01)
        {
            Eigen::Vector2d wheelPos = jointActuator->getPosition();
            bool changeDirection = computeTurningAngle(currentTurningCenter, wheelPos, steeringJS.position);
            
            double wheelSpeed = computeSpeed(currentTurningCenter, wheelPos, motionCmd_u.rotation);
            double rotationalSpeed = translateSpeedToWheelSpeed(wheelSpeed);
            wheelJS.speed = rotationalSpeed;
            if ((motionCmd_u.translation < 0) ^ changeDirection)
            {
                wheelJS.speed *= -1.;
            }
        }
        else
        {
            steeringJS.position = 0;
            wheelJS.speed = translateSpeedToWheelSpeed(motionCmd.translation);
        }
    }

    return true;
}

}