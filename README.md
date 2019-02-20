# Generalized Dubins Interval Problem (GDIP)

This repository provides the optimal solution of the GDIP wchich enables to find a tight lower-bound for the Dubins Traveling Salesman Problem with Neighborhoods (DTSPN). The provided source codes are implemented in C++11 and support the following article published at RSS 2018 conference.

```
@INPROCEEDINGS{VANA-RSS-18, 
    AUTHOR    = {Váňa, Petr and Faigl, Jan}, 
    TITLE     = {Optimal Solution of the Generalized Dubins Interval Problem}, 
    BOOKTITLE = {Proceedings of Robotics: Science and Systems}, 
    YEAR      = {2018}, 
    ADDRESS   = {Pittsburgh, Pennsylvania}, 
    MONTH     = {June}, 
    DOI       = {10.15607/RSS.2018.XIV.035} 
} 
```

This paper has been nominated for **the best student paper**.

## GDIP example

![GDIP example](https://raw.githubusercontent.com/petvana/images/master/gdip/basic-gdip-example-small.gif)

## DTRP solution

Feasible solution (blue) and the corresponding lower bound (red).

![DTRP solution](https://raw.githubusercontent.com/petvana/images/master/gdip/rss-example-small.gif)

## Basic usage in C++

Local compilation is possible by running prepared install_local.sh script. It creates these directories: include, lib, bin, build.

The following example shows how to utilize this library for finding the optimal solution of the GDIP. This example is provided in the file gdipexample.cpp.

```c++
#include <iostream>

#include "opendubins/dubins.h"

using namespace std;
using namespace opendubins;

int main() {

    cout << "Hello, this is example of optimal solution of the GDIP!" << endl;

    // Centers of two given regions
    Point p1(0,0);
    Point p2(5,5);

    // Radii of the regions
    double region1radius = 0.5;
    double region2radius = 2;

    cout << "Let have two locations: " << endl << p1 << "," << endl << p2 << "." << endl;

    cout << "Now, we define intervals of the corresponding heading angles." << endl;
    /* AngleInterval has three parameters:
     * 1) Position of the center
     * 2) Right limit of the angle interval [ 2) = minimum angle ]
     * 3) Interval size [ 2) + 3) = maximum angle ]
     */
    AngleInterval a1(p1, 0, M_PI/2);
    AngleInterval a2(p2, M_PI, M_PI/2);
    /* Thus, interval of a1 is [0, M_PI/2]
     * and nterval of a2 is [M_PI, 3*M_PI/2].
     */

    cout << "The angle intervals are: " << endl << a1 << "," << endl << a2 << "." << endl;
    cout << "The goal is to find the shortest Dubins path between two regions."
         << "The first region has radius  " << region1radius << ", and the second one " << region2radius << "." << endl;

    // Minimum turning radius
    double radius = 1.0;

    Dubins d = Dubins(a1, a2, radius, region1radius, region2radius);
    cout << "For radius=" << radius << " the shortest path (optimal GDIP solution) is:" << endl << d << endl;

    // Check if the endpoints are in the given disk-shaped regions.
    cout << "Chech the first endpoint (" << p1.distance(d.start.point) << " <= " << region1radius << ")." << endl;
    cout << "Chech the second endpoint (" << p2.distance(d.end.point) << " <= " << region2radius << ")." << endl;

    return 0;
}
```
