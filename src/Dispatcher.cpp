#include "Dispatcher.hpp"
#include "trajectory_follower/Motion2D.hpp"

using namespace motion_controller;

double Dispatcher::calcWheelSpeedWhenTurning()
{

    double steeringSpeed = 0.5;
    double wheelSpeed = steeringSpeed * geometry.wheelRadius / geometry.scrubRadius ;
    return wheelSpeed;
}

base::samples::Joints Dispatcher::compute(const trajectory_follower::Motion2D& motionCmd, base::samples::Joints &actuatorsCommand, base::samples::Joints &actuatorsFeedback)
{
    if (motionCmd.translation == 0 && motionCmd.rotation != 0) 
    {
        pointTurn->compute(motionCmd, actuatorsCommand);

    }
    else if (motionCmd.rotation == 0)
    {
        lateral->compute(motionCmd, actuatorsCommand);
    }
    else
    {
        if (!ackermann->compute(motionCmd, actuatorsCommand))
        {
            pointTurn->compute(motionCmd, actuatorsCommand);
        }
    }
   std::cout << "Hallo?" << std::endl;
   if(useFeedback)
    {
        std::cout << "WIR USEN FEEDBECK!" << std::endl;
        bool needsToWaitForTurn = false;
        for (auto jointActuator: controllerBase->getJointActuators())
        {
            JointCmd* positionCmd = jointActuator->getJointCmdForType(JointCmdType::Position);
            JointCmd* steeringCmd = jointActuator->getJointCmdForType(JointCmdType::Speed);

            if (!positionCmd || !steeringCmd)
            {
                continue;
            }

            //base::JointState &speedJS(actuatorsCommand[actuatorsCommand.mapNameToIndex(positionCmd->getName())]);
            //base::JointState &speedJS_fb(actuatorsFeedback[actuatorsFeedback.mapNameToIndex(positionCmd->getName())]);
            base::JointState &steerJS(actuatorsCommand[actuatorsCommand.mapNameToIndex(steeringCmd->getName())]);
            base::JointState &steerJS_fb(actuatorsFeedback[actuatorsFeedback.mapNameToIndex(steeringCmd->getName())]);
            
            if(steerJS.position != steerJS_fb.position){
                needsToWaitForTurn = true;
            }
            
        }
        
        
       if(needsToWaitForTurn)
        {
            for (auto jointActuator: controllerBase->getJointActuators())
            {
                JointCmd* positionCmd = jointActuator->getJointCmdForType(JointCmdType::Position);
                JointCmd* steeringCmd = jointActuator->getJointCmdForType(JointCmdType::Speed);

                if (!positionCmd || !steeringCmd)
                {
                    continue;
                }

                base::JointState &speedJS(actuatorsCommand[actuatorsCommand.mapNameToIndex(positionCmd->getName())]);
                base::JointState &speedJS_fb(actuatorsFeedback[actuatorsFeedback.mapNameToIndex(positionCmd->getName())]);
                base::JointState &steerJS(actuatorsCommand[actuatorsCommand.mapNameToIndex(steeringCmd->getName())]);
                base::JointState &steerJS_fb(actuatorsFeedback[actuatorsFeedback.mapNameToIndex(steeringCmd->getName())]);
                
                double diff = std::abs(steerJS.position - steerJS_fb.position);
                if(diff > magicAckermannThreshold / 2){
                    speedJS.speed = calcWheelSpeedWhenTurning();
                    speedJS.speed *= diff > 0 ? 1 : -1;
                }else{
                    speedJS.speed = 0;
                }
                
            }
        }
        
        
    /*if(geometry.wheelMaxVel != 0)
    {
        bool isTooFast = false;
        double softLimit = geometry.wheelMaxVel;
        map.applyGroupwise(out,WHEEL_ONLY, [&isTooFast,&softLimit](base::JointState& state){isTooFast |= std::abs(state.speed) > softLimit;});
        if(isTooFast)
        {
            double maxSpeed = 0;
            map.applyGroupwise(out,WHEEL_ONLY, [&maxSpeed](base::JointState& state){maxSpeed = std::abs(state.speed) > maxSpeed ? std::abs(state.speed) : maxSpeed;});
            double reduce = geometry.wheelMaxVel / maxSpeed;
            std::cerr << "control/spacebot_motioncontroller: SPEED is over Limit! Reducing to " << reduce*100 << "%" << std::endl;
            map.applyGroupwise(out,WHEEL_ONLY, [&reduce](base::JointState& state){state.speed *= reduce;});
        }
    }*/
    }
   
   return actuatorsCommand;
}
