#include "Ackermann.hpp"

namespace motion_controller {

double Ackermann::computeTurningAngle(const Eigen::Vector2d& turningCenter, const Eigen::Vector2d& wheelposition)
{
    Eigen::Vector2d vec = turningCenter - wheelposition;
    return atan2(vec.y(), vec.x());
}

double Ackermann::computeWheelspeed(const Eigen::Vector2d& turningCenter, const Eigen::Vector2d& wheelposition, const double& targetRotation)
{
    double radius = (turningCenter - wheelposition).norm();
    return radius*targetRotation;
}

Eigen::Vector2d Ackermann::computeTurningCenter(const trajectory_follower::Motion2D& motionCommand)
{
    double radius = std::abs(motionCommand.translation)/motionCommand.rotation;
    return Eigen::Vector2d(0., radius);
}

double Ackermann::translateSpeedToRotation(const double &speed)
{
    double surface = geometry.wheelRadius * 2 * M_PI;
    return speed / surface * M_PI;
}

const base::samples::Joints& Ackermann::compute(const trajectory_follower::Motion2D& motionCmd)
{
    for (auto jointCmd: jointCmds)
    {
        resetJoint(jointCmd);
    }

    for (auto jointActuator: jointActuators)
    {
        JointCmd* positionCmd = jointActuator->getJointCmdForType(JointCmdType::Position);
        JointCmd* steeringCmd = jointActuator->getJointCmdForType(JointCmdType::Steering);

        if (!positionCmd || !steeringCmd)
        {
            continue;
        }
        
        base::JointState &positionJointState(joints[joints.mapNameToIndex(positionCmd->getName())]);
        base::JointState &steeringJointState(joints[joints.mapNameToIndex(steeringCmd->getName())]);

        Eigen::Vector2d turningCenter = computeTurningCenter(motionCmd);
        const Eigen::Vector2d &wheelPos = jointActuator->getPosition();
        double turningAngle = computeTurningAngle(turningCenter, wheelPos);
        positionJointState.position = turningAngle;

        double wheelSpeed = computeWheelspeed(turningCenter, wheelPos, motionCmd.rotation);
        double rotationalSpeed = translateSpeedToRotation(wheelSpeed);
        steeringJointState.speed = rotationalSpeed;
    }

    return joints;
}

}
