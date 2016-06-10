#include "Lateral.hpp"

namespace motion_controller {

bool Lateral::compute(const base::commands::Motion2D &motionCmd, base::samples::Joints& actuatorsCommand)
{   
    double speed = translateSpeedToWheelSpeed(motionCmd.translation);
   
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
        steeringJointState.speed = translateSpeedToWheelSpeed(speed);
        if(motionCmd.heading.getRad() > M_PI){
            positionJointState.position -= M_PI;
            positionJointState.speed *= -1;
        }
    }

    return true;
}
    
}