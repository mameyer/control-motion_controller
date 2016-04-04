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
        this->jointActuator->registerJointCmd(this);
        this->jointActuator = jointActuator;
    }
}

const JointActuator& Controller::addJointActuator(const Eigen::Vector2d& position)
{
    JointActuator *jointActuator = new JointActuator(position);
    jointActuators.push_back(jointActuator);
    return *jointActuator;
}

const JointCmd& Controller::addJointCmd(const std::string& name, JointCmdType type)
{
    for (auto jointCmd: jointCmds)
    {
        if (jointCmd->getType() == type)
        {
            if (jointCmd->getName() != name)
            {
                throw std::runtime_error("there is allready a joint of this type available.");
            }

            return *jointCmd;
        }
    }

    JointCmd *jointCmd = new JointCmd(name, type);
    joints.names.push_back(name);
    resetJoint(jointCmd);
    jointCmds.push_back(jointCmd);
    return *jointCmd;
}

void Controller::resetJoint(JointCmd *jointCmd)
{
    base::JointState &jointState(joints[joints.mapNameToIndex(jointCmd->getName())]);
    switch (jointCmd->getType())
    {
    case JointCmdType::Position:
        jointState.position = 0.;
        break;

    case JointCmdType::Steering:
        jointState.speed = 0.;
        break;

    default:
        break;
    }
}

}