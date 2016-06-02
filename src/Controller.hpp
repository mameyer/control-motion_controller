#pragma once

#include "MotionControllerTypes.hpp"
#include <base/samples/Joints.hpp>
#include <map>
#include <vector>
#include <base/Eigen.hpp>
#include <tuple>
#include <base/commands/Motion2D.hpp>

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
    std::pair<double, double> wheelPositionAxisInvalidArea;
    double maxRotationAngle;
    
public:
    ControllerBase()
    {
        wheelPositionAxisInvalidArea.first = base::unset<double>();
        wheelPositionAxisInvalidArea.second = base::unset<double>();
        maxRotationAngle = M_PI/2;
    }
    
    JointActuator* addJointActuator(const base::Vector2d &position);
    JointCmd* addJointCmd(const std::string &name, JointCmdType type);
    void resetJoint(JointCmd *jointCmd, base::JointState &actuatorsState);
    void resetAllJoints(base::samples::Joints &actuatorsCmd);
    inline std::vector< JointActuator* > getJointActuators() { return jointActuators; };
    //void eachJoint(std::function<void(base::JointState&)>, std::function<void(base::JointState&)>);
    inline std::vector< JointCmd* > getJointCmds() { return jointCmds; };
    bool checkWheelPositionValid(const double &wheelPositionAxis);
    inline void setMaxRotationAngle(const double &maxRotationAngle) { this->maxRotationAngle = maxRotationAngle; };
    inline double getMaxRotationAngle() { return this->maxRotationAngle; };
};

class Controller 
{  
protected:
    double ackermannRatio;
    double turningCenterX;
    
    Geometry geometry;
    ControllerBase *controllerBase;
    
    double translateSpeedToWheelSpeed(const double &speed);
    bool computeTurningAngle(const Eigen::Vector2d &turningCenter, const Eigen::Vector2d &wheelposition, double &turningAngle);
    double computeSpeed(const Eigen::Vector2d &turningCenter, const Eigen::Vector2d &wheelposition,
                             const double &targetRotation);
public:
    Controller(const Geometry &geometry, ControllerBase *controllerBase)
        : geometry(geometry),
          controllerBase(controllerBase)
    {
        ackermannRatio = 0.5;
        turningCenterX = 0;
    }
    virtual ~Controller(){
    }
    
    void setAckermannRatio(double ackermannRatio);
    virtual bool compute(const base::commands::Motion2D &motionCmd, base::samples::Joints& actuatorsCommand) = 0;
};
    
}