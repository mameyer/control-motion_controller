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
    WheelMidLeft,
    WheelMidRight,
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
            case WheelMidLeft:
            case WheelRearLeft:
                wheelPosition.y() = axleTrack / 2 + scrubRadius;
                break;
            
            case WheelFrontRight:
            case WheelMidRight:
            case WheelRearRight:
                wheelPosition.y() = -axleTrack / 2 - scrubRadius;
                break;
                
            default:
                break;
        }
        
        switch (wheelType)
        {
            case WheelRearLeft:
            case WheelRearRight:
                wheelPosition.x() = -wheelbase / 2;
                break;
            
            case WheelFrontLeft:
            case WheelFrontRight:
                wheelPosition.x() = wheelbase / 2;
                break;
                
            case WheelMidLeft:    
            case WheelMidRight:
                wheelPosition.x() = 0.;
                break;
                
            default:
                break;
        }
    
        return wheelPosition;
    }
};

}