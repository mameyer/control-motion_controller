#include "PointTurn.hpp"

namespace motion_controller {

bool PointTurn::compute(const trajectory_follower::Motion2D &motionCmd, base::samples::Joints& actuatorsCommand)
{
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

        Eigen::Vector2d turningCenter(0., 0.);
        Eigen::Vector2d wheelPos = jointActuator->getPosition();
        bool changeDirection = computeTurningAngle(turningCenter, wheelPos, positionJointState.position);
        double wheelSpeed = computeWheelspeed(turningCenter, wheelPos, motionCmd.rotation);
        double rotationalSpeed = translateSpeedToRotation(wheelSpeed);
        steeringJointState.speed = rotationalSpeed;

        if ((motionCmd.translation < 0) ^ changeDirection)
        {
            steeringJointState.speed *= -1.;
        }
    }

    return true;
}

}
