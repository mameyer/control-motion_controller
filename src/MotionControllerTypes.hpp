#pragma once

#include <base/Eigen.hpp>

namespace motion_controller
{
   
enum JointCmdType
{
    Position,
    Speed
};
   
enum DriveMode
{
    ModeAckermann,
    ModeLateral,
    ModeTurnOnSpot
};

enum WheelType
{
    WheelFrontLeft,
    WheelFrontRight,
    WheelRearLeft,
    WheelRearRight,
    WheelOther
};

struct Geometry
{
    /** Axle Track B (in meters) = distance between the steer axis of 2 wheels from the same axle  */
    float axleTrack;

    /** Wheelbase L (in meters) = distance between the front axle and the axis of rotation of the rear bogies */
    float wheelbase;

    /** Offset D (in meters) = for the rear wheels, distance between the axis of rotation of the bogies and the steer axis of the wheel */
    float wheelOffset;

    /** Scrub radius d (in meters) = distance between the steer axis of the wheel and its middle plane */
    float scrubRadius;

    /** Wheel radius R (in meters) = radius of the wheels */
    float wheelRadius;
    
    /** maximal velocity of Wheels in m/s */
    float wheelMaxVel;
    
    Geometry()
        : axleTrack(0),
          wheelbase(0),
          wheelOffset(0),
          scrubRadius(0),
          wheelRadius(0),
          wheelMaxVel(0)
    {
    }
    
    base::Vector2d getWheelPosition(WheelType wheelType)
    {
        base::Vector2d wheelPosition;
        switch (wheelType)
        {
            case WheelFrontLeft:
                wheelPosition.x() = wheelbase / 2;
                wheelPosition.y() = axleTrack / 2;
                break;
                
            case WheelFrontRight:
                wheelPosition.x() = wheelbase / 2;
                wheelPosition.y() = -axleTrack / 2;
                break;
                
            case WheelRearLeft:
                wheelPosition.x() = -wheelbase / 2;
                wheelPosition.y() = axleTrack / 2;
                break;
                
            case WheelRearRight:
                wheelPosition.x() = -wheelbase / 2;
                wheelPosition.y() = -axleTrack / 2;
                break;
                
            default:
                break;
        }
        
        return wheelPosition;
    }
};

}