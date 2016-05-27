#include "Lateral.hpp"

namespace motion_controller {

bool Lateral::compute(const base::commands::Motion2D &motionCmd, base::samples::Joints& actuatorsCommand)
{   
    double speed = translateSpeedToRotation(motionCmd.translation);
   
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
        
        positionJointState.position = motionCmd.heading.getRad();
        steeringJointState.speed = speed;
    }

    return true;
}
    
}