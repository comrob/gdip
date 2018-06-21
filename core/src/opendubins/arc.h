/**
 * @file arc.h
 * @author Petr Vana
 * @brief Arc is a basic segment in Dubins maneuver
 */

#pragma once

#include "path.h"
#include "line.h"
#include "stateatdistance.h"

namespace opendubins {

    class Arc : public Path {
    public:

        State state;
        double angle;
        double radius;

        Arc() :
                angle(NAN), radius(NAN) {
        }

        Arc(const State &state, double angle, double radius) :
                state(state), angle(angle), radius(radius) {
        }

        virtual State getState(double len) const;

        virtual StateAtDistance getClosestStateAndDistance(const Point &p) const;

        StateAtDistance intersectionPoint(const Line &line) const;

        Point getCenter() const;

        State getStateByAngle(double angle) const;

        State getEnd() const;

        inline double getLength() const {
            return std::fabs(getAngle()) * getRadius();
        }

        inline double getAngle() const { return angle; }

        inline double getRadius() const { return radius; }

        inline State getStart() const { return state; }
    };

    inline std::ostream &operator<<(std::ostream &os, const Arc &a) {
        os << "Arc: from={" << a.state << "} angle=" << a.angle << " RADIUS=" << a.radius;
        return os;
    }

}
