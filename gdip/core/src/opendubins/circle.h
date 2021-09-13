/**
 * @file circle.h
 * @author Petr Vana
 * @brief Circle class used for calculating intersections, etc.
 */

#pragma once

#include <tuple>

#include "state.h"
#include "arc.h"
#include "angleinterval.h"

namespace opendubins {

    struct Circle {

        Point center;
        double radius;

        inline Circle() : center(Point(GDIP_NAN, GDIP_NAN)), radius(GDIP_NAN) { };

        Circle(const Point &center, const double &radius);

        virtual ~Circle();

        bool pointInCircle(const Point&) const;

        Point getCenter() const;

        double perimeterLength() const;

        std::tuple<int, Point, Point> halfLineIntersections(const State& pos) const;
        Point halfLineIntersection(const State& pos) const;

        inline double getRadius() { return radius; }

        std::tuple<State, double> firstIntersection(const Arc& arc) const;

        // find circle intersection interval
        // return AngleInterval which stands for intersection arc of *this circle and starts in *this.center
        // if there is no intersection - invalid interval is retuned (see AngleInterval::isValid())
        // if the first circle is completely in the other one [0,2PI] interval is returned
        AngleInterval getIntersectionAngleInterval(const Circle& other) const;

        // find circles intersection interval
        // intervals are merged - sum of AngleInterval
        std::vector<AngleInterval> getIntersectionAngleIntervals(const std::vector<Circle> circles) const;

        // find the shortest vector from the sector.point
        // wanted vector need to intersect both the circle and the sector
        Vector shortestVectorInSector(const AngleInterval& sector) const;

        // find the closest point from the sector.point
        // wanted point need to intersect both the circle and the sector
        Point closestPointInSector(const AngleInterval& sector) const;
    };

}
