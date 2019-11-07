//
//  flightplan.cpp
//  twopoints
//
//  Created by Harvard Virgil Humphrey on 2019/11/6.
//  Copyright © 2019 Harvard Virgil Humphrey. All rights reserved.
//

#include <iostream>
#include <stdio.h>
#include <vector>
#include "flightplan.hpp"
#include "math.h"

missionplan createflightplan(double distance,double target_height)    {
    //std::cout<<C_PI<<std::endl;
    missionplan flightplan;
    double m = 2;
    vector3D waypoint1,waypoint2;
    waypoint1.x = 0;
    waypoint1.y = 0;
    waypoint1.z = 2;
    flightplan.waypts.push_back(waypoint1);

    waypoint2.x = 0;
    waypoint2.y = distance;
    waypoint2.z = 2;
    flightplan.waypts.push_back(waypoint2);
    flightplan.nwaypoints = m;
    return flightplan;
}
