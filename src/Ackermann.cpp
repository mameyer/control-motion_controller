#include "Ackermann.hpp"

namespace motion_controller {

bool Ackermann::computeTurningAngle(const Eigen::Vector2d &turningCenter, const Eigen::Vector2d &wheelposition, double &turningAngle)
{
    bool changeDirection = false;
    Eigen::Vector2d vec = wheelposition - turningCenter;
    vec = Eigen::Rotation2Dd(M_PI/2) * vec;
    turningAngle = atan(vec.y()/vec.x());
    if (std::signbit(turningAngle) != std::signbit(atan2(vec.y(), vec.x())))
    {
        changeDirection = true;
    }

    return changeDirection;
}
    
Eigen::Vector2d Ackermann::computeTurningCenter(const base::commands::Motion2D& motionCommand)
{
    double radius = std::abs(motionCommand.translation)/motionCommand.rotation;
    Eigen::Vector2d vec(turningCenterX, radius);
    Eigen::Rotation2Dd rot(motionCommand.heading.getRad());
    return rot*vec;
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

        if (motionCmd_u.rotation != 0)
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
        }else{
            throw new std::invalid_argument("Invalid Input for Ackermann, Rotation is Zero. [BUG] probably in Dispatcher");
        }
    }

    return true;
}

}