#include "Ackermann.hpp"

namespace motion_controller {

double Ackermann::computeTurningAngle(const Eigen::Vector2d& turningCenter, const Eigen::Vector2d& wheelposition)
{
    Eigen::Vector2d vec = wheelposition - turningCenter;
    std::cout << "wheelposition: " << wheelposition.transpose() << ", " << "turningCenter: " << turningCenter.transpose() << std::endl;
    vec = Eigen::Rotation2Dd(M_PI/2) * vec;
    std::cout << "vec: " << vec.transpose() << std::endl;
    double angle = atan2(vec.y(), vec.x());
    std::cout << "angle: " << angle << std::endl;
    return angle;
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
            std::cout << "wheelPos: " << wheelPos.transpose() << std::endl;
            double turningAngle = computeTurningAngle(currentTurningCenter, wheelPos);
            double turningAngleN = base::Angle::normalizeRad(turningAngle);
            if (turningAngleN > M_PI/2 || turningAngleN <= -M_PI/2)
            {
                double intp;
                const double side = copysign(M_PI/2, turningAngleN);
                turningAngleN = -side + M_PI * modf((turningAngleN-side) / M_PI, &intp);
            }
            positionJointState.position = turningAngleN;
            std::cout << "turningAngleN: " << turningAngleN << std::endl;

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