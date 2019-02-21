#include "gdipJuliaAPI.h"
#include <math.h>
#include <stdio.h>

/*************************
  GDIP interface
**************************/

GdipJuliaAPI juliaAPI;

double julia_set_configurations(
    double p1x, double p1y, double t1,
    double p2x, double p2y, double t2,
    double radius)
{
    State q1(p1x, p1y, t1);
    State q2(p2x, p2y, t2);
    juliaAPI.maneuver = Dubins(q1, q2, radius);
    return juliaAPI.maneuver.length;
}

double julia_set_configurations_dip(
    double p1x, double p1y, double t1left, double t1diff, 
    double p2x, double p2y, double t2left, double t2diff,  
    double radius)
{
    AngleInterval int1(Point(p1x, p1y), t1left, t1diff);
    AngleInterval int2(Point(p2x, p2y), t2left, t2diff);
    juliaAPI.maneuver = Dubins(int1, int2, radius);
    return juliaAPI.maneuver.length;
}

double julia_set_configurations_gdip(
    double p1x, double p1y, double t1left, double t1diff, double radius1,
    double p2x, double p2y, double t2left, double t2diff, double radius2,
    double radius
)
{
    AngleInterval int1(Point(p1x, p1y), t1left, t1diff);
    AngleInterval int2(Point(p2x, p2y), t2left, t2diff);
    juliaAPI.maneuver = Dubins(int1, int2, radius, radius1, radius2);
    return juliaAPI.maneuver.length;
}

double julia_get_length(){
    return juliaAPI.maneuver.length;
}

void julia_sample_state_to_tmp(double distance){
    juliaAPI.tmp_state = juliaAPI.maneuver.getState(distance);
}

double julia_get_tmp_x(){
    return juliaAPI.tmp_state.point.x;
}

double julia_get_tmp_y(){
    return juliaAPI.tmp_state.point.y;
}

double julia_get_tmp_theta(){
    return juliaAPI.tmp_state.ang;
}
