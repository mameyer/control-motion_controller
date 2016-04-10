#include "PointTurn.hpp"

namespace motion_controller {

const base::samples::Joints& PointTurn::compute(const trajectory_follower::Motion2D& motionCmd)
{
    controllerBase->resetAllJoints();
    base::samples::Joints &joints(controllerBase->getJoints());

    for (auto jointActuator: controllerBase->getJointActuators())
    {
        JointCmd* positionCmd = jointActuator->getJointCmdForType(JointCmdType::Position);
        JointCmd* steeringCmd = jointActuator->getJointCmdForType(JointCmdType::Speed);

        if (!positionCmd || !steeringCmd)
        {
            continue;
        }

        base::JointState &positionJointState(joints[joints.mapNameToIndex(positionCmd->getName())]);
        base::JointState &steeringJointState(joints[joints.mapNameToIndex(steeringCmd->getName())]);

        Eigen::Vector2d turningCenter(0., 0.);
        Eigen::Vector2d wheelPos = jointActuator->getPosition();
        double turningAngle = computeTurningAngle(turningCenter, wheelPos);
        double turningAngleN = base::Angle::normalizeRad(turningAngle);
        if (turningAngleN > M_PI/2 || turningAngleN <= -M_PI/2)
        {
            double intp;
            const double side = copysign(M_PI/2, turningAngleN);
            turningAngleN = -side + M_PI * modf((turningAngleN-side) / M_PI, &intp);
        }
        positionJointState.position = turningAngleN;

        double wheelSpeed = computeWheelspeed(turningCenter, wheelPos, motionCmd.rotation);
        double rotationalSpeed = translateSpeedToRotation(wheelSpeed);
        steeringJointState.speed = rotationalSpeed * (turningAngle != turningAngleN ? -1 : 1);

        if (motionCmd.translation < 0)
        {
            steeringJointState.speed *= -1.;
        }
    }

    return joints;
}

}