#include "Controller.hpp"

namespace motion_controller {

JointCmd* JointActuator::getJointCmdForType(JointCmdType type)
{
    for (auto registeredJointCmd: registeredJointCmds)
    {
        if (registeredJointCmd->getType() == type)
        {
            return registeredJointCmd;
        }
    }

    return nullptr;
}

void JointActuator::registerJointCmd(JointCmd* jointCmd)
{
    if (getJointCmdForType(jointCmd->getType()) != nullptr)
    {
        std::runtime_error("joint with same type already registered.");
    }

    registeredJointCmds.push_back(jointCmd);
}

bool JointCmd::isRegistered()
{
    if (jointActuator != nullptr)
    {
        return true;
    }

    return false;
}

void JointCmd::registerAt(JointActuator* jointActuator)
{
    if (!isRegistered())
    {
        this->jointActuator = jointActuator;
        this->jointActuator->registerJointCmd(this);
    }
}

JointActuator* ControllerBase::addJointActuator(const base::Vector2d& position)
{
    jointActuators.resize(jointActuators.size()+1);
    jointActuators[jointActuators.size()-1] = new JointActuator(position);
    std::cout << "addJointActuator: position = " << jointActuators.back()->getPosition().transpose() << std::endl;

    if (base::isUnset<double>(wheelPositionAxisInvalidArea.first))
    {
        wheelPositionAxisInvalidArea.first = std::numeric_limits< double >::max();
    }

    if (jointActuators.back()->getPosition().y() < wheelPositionAxisInvalidArea.first)
    {
        wheelPositionAxisInvalidArea.first = jointActuators.back()->getPosition().y();
    }

    if (base::isUnset<double>(wheelPositionAxisInvalidArea.second))
    {
        wheelPositionAxisInvalidArea.second = std::numeric_limits< double >::min();
    }

    if (jointActuators.back()->getPosition().y() > wheelPositionAxisInvalidArea.second)
    {
        wheelPositionAxisInvalidArea.second = jointActuators.back()->getPosition().y();
    }

    return jointActuators.back();
}

JointCmd* ControllerBase::addJointCmd(const std::string& name, JointCmdType type)
{
    for (auto jointCmd: jointCmds)
    {
        if (jointCmd->getType() == type && jointCmd->getName() == name)
        {
            throw std::runtime_error("there is allready a joint with the same name and type available.");
        }
    }

    jointCmds.resize(jointCmds.size()+1);
    jointCmds[jointCmds.size()-1] = new JointCmd(name, type);
    return jointCmds.back();
}

void ControllerBase::resetJoint(JointCmd *jointCmd, base::JointState &actuatorsState)
{
    switch (jointCmd->getType())
    {
    case JointCmdType::Position:
        actuatorsState.position = 0.;
        break;

    case JointCmdType::Speed:
        actuatorsState.speed = 0.;
        break;

    default:
        break;
    }
}

void ControllerBase::resetAllJoints(base::samples::Joints &actuatorsCmd)
{
    actuatorsCmd.clear();
    actuatorsCmd.resize(jointCmds.size());
    
    for (unsigned int i=0; i<jointCmds.size(); i++)
    {
        actuatorsCmd.names[i] = jointCmds.at(i)->getName();
        resetJoint(jointCmds.at(i), actuatorsCmd[i]);
    }
}

bool ControllerBase::checkWheelPositionValid(const double &wheelPositionAxis)
{
    if (!base::isUnset<double>(wheelPositionAxisInvalidArea.first) && !base::isUnset<double>(wheelPositionAxisInvalidArea.second))
    {
        if (wheelPositionAxis >= wheelPositionAxisInvalidArea.first && wheelPositionAxis <= wheelPositionAxisInvalidArea.second)
        {
            std::cout << "wheel position invalid." << std::endl;
            return false;
        }
    }

    return true;
}

void Controller::setAckermannRatio(double ackermannRatio)
{
    this->ackermannRatio = ackermannRatio;
    turningCenterX = 0.5*geometry.wheelbase * (2*this->ackermannRatio-1);
    std::cout << "setAckermannRatio: " << this->ackermannRatio << ", turningCenterX: " << turningCenterX << std::endl;
}

double Controller::translateSpeedToRotation(const double &speed)
{
    double surface = geometry.wheelRadius * 2 * M_PI;
    return speed / surface * M_PI;
}

bool Controller::computeTurningAngle(const Eigen::Vector2d &turningCenter, const Eigen::Vector2d &wheelposition, double &turningAngle)
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

double Controller::computeWheelspeed(const Eigen::Vector2d& turningCenter, const Eigen::Vector2d& wheelposition, const double& targetRotation)
{
    double radius = (turningCenter - wheelposition).norm();
    return radius*targetRotation;
}

}