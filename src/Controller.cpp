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
        std::runtime_error("joint with same type allready registered.");
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
    
    joints.resize(joints.size()+1);
    joints.names[joints.size()-1] = name;
    jointCmds.resize(jointCmds.size()+1);
    jointCmds[jointCmds.size()-1] = new JointCmd(name, type);
    resetJoint(jointCmds.back());
    return jointCmds.back();
}

void ControllerBase::resetJoint(JointCmd *jointCmd)
{
    base::JointState &jointState(joints[joints.mapNameToIndex(jointCmd->getName())]);
    switch (jointCmd->getType())
    {
    case JointCmdType::Position:
        jointState.position = 0.;
        break;

    case JointCmdType::Speed:
        jointState.speed = 0.;
        break;

    default:
        break;
    }
}

void ControllerBase::resetAllJoints()
{
    for (auto jointCmd: jointCmds)
    {
        resetJoint(jointCmd);
    }
}

double Controller::translateSpeedToRotation(const double &speed)
{
    double surface = geometry.wheelRadius * 2 * M_PI;
    return speed / surface * M_PI;
}

double Controller::computeTurningAngle(const Eigen::Vector2d& turningCenter, const Eigen::Vector2d& wheelposition)
{
    Eigen::Vector2d vec = wheelposition - turningCenter;
    vec = Eigen::Rotation2Dd(M_PI/2) * vec;
    double angle = atan2(vec.y(), vec.x());
    return angle;
}

double Controller::computeWheelspeed(const Eigen::Vector2d& turningCenter, const Eigen::Vector2d& wheelposition, const double& targetRotation)
{
    double radius = (turningCenter - wheelposition).norm();
    return radius*targetRotation;
}

}