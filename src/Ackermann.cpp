#include "Ackermann.hpp"

namespace motion_controller {

void Ackermann::setAckermannRatio(double ackermannRatio)
{
    this->ackermannRatio = ackermannRatio;
    turningCenterX = 0.5*geometry.wheelbase * (1 - 2*this->ackermannRatio);
    std::cout << "setAckermannRatio: " << this->ackermannRatio << ", turningCenterX: " << turningCenterX << std::endl;
}

Eigen::Vector2d Ackermann::computeTurningCenter(const trajectory_follower::Motion2D& motionCommand)
{
    double radius = std::abs(motionCommand.translation)/motionCommand.rotation;
    return Eigen::Vector2d(turningCenterX, radius);
}

const base::samples::Joints& Ackermann::compute(const trajectory_follower::Motion2D& motionCmd)
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

        std::cout << "motionCmd.rotation: " << motionCmd.rotation << std::endl;
        if (motionCmd.rotation > 0.01 || motionCmd.rotation < -0.01)
        {
            currentTurningCenter = computeTurningCenter(motionCmd);
            Eigen::Vector2d wheelPos = jointActuator->getPosition();
            double turningAngle = computeTurningAngle(currentTurningCenter, wheelPos);
            double turningAngleN = base::Angle::normalizeRad(turningAngle);
            if (turningAngleN > M_PI/2 || turningAngleN <= -M_PI/2)
            {
                double intp;
                const double side = copysign(M_PI/2, turningAngleN);
                turningAngleN = -side + M_PI * modf((turningAngleN-side) / M_PI, &intp);
            }
            positionJointState.position = turningAngleN;

            double wheelSpeed = computeWheelspeed(currentTurningCenter, wheelPos, motionCmd.rotation);
            double rotationalSpeed = translateSpeedToRotation(wheelSpeed);
            steeringJointState.speed = rotationalSpeed * (turningAngle != turningAngleN ? -1 : 1);
            
            if (motionCmd.translation < 0)
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

    return joints;
}

}