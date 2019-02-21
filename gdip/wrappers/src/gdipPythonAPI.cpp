#include "gdipPythonAPI.h"
#include <math.h>
#include <stdio.h>

/*************************
  GDIP interface
**************************/

GdipPythonAPI* new_GdipAPI(){
  return new GdipPythonAPI();
}

void python_set_configurations(GdipPythonAPI* api, double start[3], double end[3], double radius){
  State q1(start[0], start[1], start[2]);
  State q2(end[0], end[1], end[2]);
  api->maneuver = Dubins(q1, q2, radius);
}

void python_set_configurations_dip(GdipPythonAPI* api, double point1[2], double interval1[2], double point2[2], double interval2[2], double radius){
  AngleInterval int1(Point(point1[0], point1[1]), interval1[0], interval1[1]);
  AngleInterval int2(Point(point2[0], point2[1]), interval2[0], interval2[1]);
  api->maneuver = Dubins(int1, int2, radius);
}

void python_set_configurations_gdip(GdipPythonAPI* api, double point1[2], double interval1[2], double radius1, double point2[2], double interval2[2], double radius2, double radius){
  AngleInterval int1(Point(point1[0], point1[1]), interval1[0], interval1[1]);
  AngleInterval int2(Point(point2[0], point2[1]), interval2[0], interval2[1]);
  api->maneuver = Dubins(int1, int2, radius, radius1, radius2);
}


double python_get_length(GdipPythonAPI* api){
  return api->maneuver.length;
}

void python_sample_state_to_tmp(GdipPythonAPI* api, double distance){
  api->tmp_state = api->maneuver.getState(distance);
}

double python_get_tmp_x(GdipPythonAPI* api){
  return api->tmp_state.point.x;
}

double python_get_tmp_y(GdipPythonAPI* api){
  return api->tmp_state.point.y;
}

double python_get_tmp_theta(GdipPythonAPI* api){
  return api->tmp_state.ang;
}

void python_destruct(GdipPythonAPI* api){
  delete api;
}
