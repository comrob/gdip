#pragma once 

#include "opendubins/dubins.h"

using namespace opendubins;

struct GdipPythonAPI{
    Dubins maneuver;   
    State tmp_state;
};


#ifdef __cplusplus
extern "C" {
#endif

    GdipPythonAPI* new_GdipAPI();

    void python_set_configurations(GdipPythonAPI* api, double start[3], double end[3], double radius);
    void python_set_configurations_dip(GdipPythonAPI* api, double point1[2], double interval1[2], double point2[2], double interval2[2], double radius);
    void python_set_configurations_gdip(GdipPythonAPI* api, double point1[2], double interval1[2], double radius1, double point2[2], double interval2[2], double radius2, double radius);

    double python_get_length(GdipPythonAPI* api);
  
    void python_sample_state_to_tmp(GdipPythonAPI* api, double distance);

    double python_get_tmp_x(GdipPythonAPI* api);
    double python_get_tmp_y(GdipPythonAPI* api);
    double python_get_tmp_theta(GdipPythonAPI* api);

    void python_destruct(GdipPythonAPI* api);

#ifdef __cplusplus
}
#endif

