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
    
    Eigen::Vector3d direction = Eigen::Vector3d(x,y,0);  
    double rotation = base::Angle::vectorToVector(direction, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitZ()).getRad();
   
    if (rotation > M_PI_2)
    {
        rotation -= M_PI;
        //speed *= -1;
    }
    else if (rotation < -M_PI_2)
    {
        rotation += M_PI;
        //speed *= -1;
    }
    
    if (base::isNaN(rotation))
    {
        rotation = 0;
    }

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