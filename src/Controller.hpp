#pragma once

#include "MotionControllerTypes.hpp"
#include <trajectory_follower/Motion2D.hpp>
#include <base/samples/Joints.hpp>
#include <map>
#include <vector>
#include <base/Eigen.hpp>

namespace motion_controller
{
    
class JointCmd;
class JointActuator;
class ControllerBase;
class Controller;

class JointActuator
{
private:
    base::Vector2d position;
    std::vector< JointCmd* > registeredJointCmds;
    
public:
    JointActuator(const base::Vector2d& position)
        : position(position)
    {
    }
    
    JointCmd* getJointCmdForType(JointCmdType type);
    void registerJointCmd(JointCmd *jointCmd);
    inline base::Vector2d getPosition() { return position; };
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
    inline std::string getName() { return name; };
    inline JointCmdType getType() { return type; };
    inline const JointActuator *getJointActuator() { return jointActuator; };
};
 
class ControllerBase
{   
protected:
    std::vector< JointActuator* > jointActuators;
    std::vector< JointCmd* > jointCmds;
    base::samples::Joints joints;
    
public:
    ControllerBase()
    {   
    }
    
    JointActuator* addJointActuator(const base::Vector2d &position);
    JointCmd* addJointCmd(const std::string &name, JointCmdType type);
    void resetJoint(JointCmd *jointCmd);
    void resetAllJoints();
    inline std::vector< JointActuator* > getJointActuators() { return jointActuators; };
    inline std::vector< JointCmd* > getJointCmds() { return jointCmds; };
    inline base::samples::Joints& getJoints() { return joints; };
};

class Controller 
{  
protected:
    Geometry geometry;
    ControllerBase *controllerBase;
    
public:
    Controller(const Geometry &geometry, ControllerBase *controllerBase)
        : geometry(geometry),
          controllerBase(controllerBase)
    {
    }
    
    virtual const base::samples::Joints& compute(const trajectory_follower::Motion2D &motionCmd) =0;
};
    
}