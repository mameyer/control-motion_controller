#include "Lateral.hpp"

namespace motion_controller {
    
double Lateral::translateSpeedToRotation(double speed)
{
    double surface = geometry.wheelRadius * 2 * M_PI;
    return speed / surface * M_PI;
}

const base::samples::Joints& Lateral::compute(const trajectory_follower::Motion2D& motionCmd)
{
    controllerBase->resetAllJoints();
    base::samples::Joints &joints(controllerBase->getJoints());
    
    double x = motionCmd.translation;
    double y = motionCmd.rotation;
    double speed = translateSpeedToRotation(motionCmd.translation);
    
    Eigen::Vector2d direction(x,y);
    
    double rotation = 0.;
    computeTurningAngle(Eigen::Vector2d(0., 0.), direction, rotation);
   
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
        
        positionJointState.position = rotation;
        steeringJointState.speed = speed;
    }

    return joints;
}
    
}