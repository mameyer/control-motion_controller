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
        JointCmd* positionCmd = jointActuator->getJointCmdForType(JointCmdType::Position);
        JointCmd* steeringCmd = jointActuator->getJointCmdForType(JointCmdType::Speed);

        if (!positionCmd || !steeringCmd)
        {
            continue;
        }

        base::JointState &positionJointState(actuatorsCommand[actuatorsCommand.mapNameToIndex(positionCmd->getName())]);
        base::JointState &steeringJointState(actuatorsCommand[actuatorsCommand.mapNameToIndex(steeringCmd->getName())]);

        if (motionCmd.rotation > 0.01 || motionCmd.rotation < -0.01)
        {
            Eigen::Vector2d wheelPos = jointActuator->getPosition();
            bool changeDirection = computeTurningAngle(currentTurningCenter, wheelPos, positionJointState.position);
            
            if (std::abs(positionJointState.position) > controllerBase->getMaxRotationAngle()) {
                std::cout << "steering angle limits reached.." << std::endl;
                return false;
            }
            
            double wheelSpeed = computeWheelspeed(currentTurningCenter, wheelPos, motionCmd.rotation);
            double rotationalSpeed = translateSpeedToRotation(wheelSpeed);
            steeringJointState.speed = rotationalSpeed;
            if (motionCmd.translation < 0 ^ changeDirection)
            {
                steeringJointState.speed *= -1.;
            }
        }
        else
        {
            positionJointState.position = 0;
            steeringJointState.speed = translateSpeedToRotation(motionCmd.translation);
        }
    }

    return true;
}

}