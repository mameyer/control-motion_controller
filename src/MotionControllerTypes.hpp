#pragma once

namespace motion_controller
{

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
};

}
