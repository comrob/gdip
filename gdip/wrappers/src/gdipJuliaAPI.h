#pragma once 

#include "opendubins/dubins.h"

using namespace opendubins;

struct GdipJuliaAPI{
    Dubins maneuver;   
    State tmp_state;
};

#ifdef __cplusplus
extern "C" {
#endif

    double julia_set_configurations(
        double p1x, double p1y, double t1, 
        double p2x, double p2y, double t2, 
        double radius
    );

    double julia_set_configurations_dip(
        double p1x, double p1y, double t1left, double t1diff, 
        double p2x, double p2y, double t2left, double t2diff,  
        double radius
    );

    double julia_set_configurations_gdip(
        double p1x, double p1y, double t1left, double t1diff, double radius1,
        double p2x, double p2y, double t2left, double t2diff, double radius2,
        double radius
    );

    double julia_get_length();
  
    void julia_sample_state_to_tmp(double distance);

    double julia_get_tmp_x();
    double julia_get_tmp_y();
    double julia_get_tmp_theta();

    bool julia_dubins_circle_intersection(
        double circle_origin_x, double circle_origin_y, double circle_radius
    );

    void julia_dubins_closest(
        double point_x, double point_y
    );

#ifdef __cplusplus
}
#endif

