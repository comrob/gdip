/**
 * @file line.h
 * @author Petr Vana
 * @brief Line is a basic segment in Dubins maneuver
 */

#pragma once

#include "path.h"
#include "stateatdistance.h"

namespace opendubins {

    class Line : public Path {
    public:

        Point p1;
        Point p2;

        inline Line() { };

        Line(const Point &, const Point &);

        Line(State, double);

        virtual State getState(double len) const;

        virtual StateAtDistance getClosestStateAndDistance(const Point &p) const;

        inline double getLength() const {
            return (p2 - p1).length();
        }

        inline Vector getDirection() const {
            return p2 - p1;
        }

        bool intersect(Line line) const;

        StateAtDistance intersectionPoint(Line line) const;

    };

    inline std::ostream &operator<<(std::ostream &os, const Line &d) {
        os << "Line from " << d.p1 << " to " << d.p2;
        return os;
    }

}

