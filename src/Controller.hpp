#pragma once

#include "MotionControllerTypes.hpp"
#include <trajectory_follower/Motion2D.hpp>
#include <base/samples/Joints.hpp>
#include <map>
#include <vector>

namespace motion_controller
{
    
class JointCmd;
class JointActuator;
   
enum JointCmdType
{
    Position, Steering
};

class JointActuator
{
private:
    const Eigen::Vector2d &position;
    std::vector< JointCmd* > registeredJointCmds;
    
public:
    JointActuator(const Eigen::Vector2d& position)
        : position(position)
    {
    }
    
    const JointCmd* getJointCmdForType(JointCmdType type);
    void registerJointCmd(JointCmd *jointCmd);
    inline const Eigen::Vector2d &getPosition() const { return position; };
    inline std::vector< JointCmd* > getRegisteredJointCmds() { return registeredJointCmds; };
};

class JointCmd
{
private:
    std::string name;
    JointCmdType type;
    JointActuator *jointActuator;
    
public:
    JointCmd(const std::string& name, JointCmdType type)
        : name(name),
          type(type),
          jointActuator(nullptr)
    {
    }
    
    void registerAt(JointActuator *jointActuator);
    bool isRegistered();
    inline const std::string& getName() { return name; };
    inline JointCmdType getType() { return type; };
    inline const JointActuator *getJointActuator() { return jointActuator; };
};
 
class Controller 
{  
protected:
    const Geometry &geometry;
    base::samples::Joints joints;
    std::vector< JointActuator* > jointActuators;
    std::vector< JointCmd* > jointCmds;
    
public:
    Controller(const Geometry &geometry)
        : geometry(geometry)
    {
    }
    
    const JointActuator &addJointActuator(const Eigen::Vector2d &position);
    const JointCmd &addJointCmd(const std::string &name, JointCmdType type);
    
    void resetJoint(JointCmd *jointCmd);
    
    virtual const base::samples::Joints& compute(const trajectory_follower::Motion2D &motionCmd) =0;
};
    
}