#include "Ackermann.hpp"

namespace motion_controller {

Eigen::Vector2d Ackermann::computeTurningCenter(const trajectory_follower::Motion2D& motionCommand)
{
    double radius = std::abs(motionCommand.translation)/motionCommand.rotation;
    return Eigen::Vector2d(turningCenterX, radius);
}

bool Ackermann::compute(const trajectory_follower::Motion2D &motionCmd, base::samples::Joints& actuatorsCommand)
{   
    currentTurningCenter = computeTurningCenter(motionCmd);
    if (!controllerBase->checkWheelPositionValid(currentTurningCenter.y()))
    {
        std::cout << "steering angle limits reached.." << std::endl;
        return false;
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

        if (motionCmd.rotation > 0.01 || motionCmd.rotation < -0.01)
        {
            Eigen::Vector2d wheelPos = jointActuator->getPosition();
            bool changeDirection = computeTurningAngle(currentTurningCenter, wheelPos, steeringJS.position);
            
            if (std::abs(steeringJS.position) > controllerBase->getMaxRotationAngle()) {
                std::cout << "steering angle limits reached.." << std::endl;
                return false;
            }
            
            double wheelSpeed = computeWheelspeed(currentTurningCenter, wheelPos, motionCmd.rotation);
            double rotationalSpeed = translateSpeedToRotation(wheelSpeed);
            wheelJS.speed = rotationalSpeed;
            if ((motionCmd.translation < 0) ^ changeDirection)
            {
                wheelJS.speed *= -1.;
            }
        }
        else
        {
            steeringJS.position = 0;
            wheelJS.speed = translateSpeedToRotation(motionCmd.translation);
        }
    }

    return true;
}

}
