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

double Ackermann::computeSpeed(const Eigen::Vector2d& turningCenter, const Eigen::Vector2d& wheelposition, const double& targetRotation)
{
    double radius = (turningCenter - wheelposition).norm();
    return radius*targetRotation;
}

bool Ackermann::compute(const base::commands::Motion2D& motionCmd, base::samples::Joints& actuatorsCommand)
{   
    base::commands::Motion2D motionCmd_u = motionCmd;
    currentTurningCenter = computeTurningCenter(motionCmd_u);
    if (isTurningCenterInside(currentTurningCenter))
    {
        motionCmd_u.translation = 0;
        currentTurningCenter = computeTurningCenter(motionCmd_u);
    }

    if (motionCmd_u.rotation != 0)
    {

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


            Eigen::Vector2d wheelPos = jointActuator->getPosition();
            bool changeDirection = computeTurningAngle(currentTurningCenter, wheelPos, steeringJS.position);
            wheelPos = jointActuator->getPrecisePosition(geometry.scrubRadius, steeringJS.position);
            double wheelSpeed = computeSpeed(currentTurningCenter, wheelPos, motionCmd_u.rotation);
            std::cout << steeringCmd->getName() << std::endl; 
            if(std::abs(wheelPos.x()) < 0.001 && motionCmd_u.translation == 0){
                //Weird bugfix of weird Speed difference
                //Basically, when point turning, scrub radius is now applied in other direction
                double scrub = wheelPos.y() > 0 ? -geometry.scrubRadius : geometry.scrubRadius;
                Eigen::Vector2d wheeloffs(0, 2*scrub);
                Eigen::Rotation2Dd rot(steeringJS.position);
                wheeloffs = rot * wheeloffs;
                wheelSpeed = computeSpeed(currentTurningCenter,  wheelPos + wheeloffs, motionCmd_u.rotation);
            }
            double rotationalSpeed = translateSpeedToWheelSpeed(wheelSpeed);
            wheelJS.speed = rotationalSpeed;
            if ((motionCmd_u.translation < 0) ^ changeDirection)
            {
                wheelJS.speed *= -1.;
            }
        }
    }else{
        throw new std::invalid_argument("Invalid Input for Ackermann, Rotation is Zero. [BUG] probably in Dispatcher");
    }

    return true;
}

}