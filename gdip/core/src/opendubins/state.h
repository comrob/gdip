/**
 * @file state.h
 * @author Petr Vana
 * @brief State of the Dubins vehicle (position (x,y) and heading angle \theta)
 */

#pragma once

#include "point.h"

namespace opendubins {

    class State final {
    public:
        Point point;
        double ang;

        State();

        State(Point, double);

        State(double, double, double);

        inline void set(double x, double y, double angle) {
            point.x = x;
            point.y = y;
            ang = angle;
        }

        void random(double);

        Vector getNormalizedDirection() const;

        inline bool invalid() {
            return std::isnan(ang);
        }

        static State getInvalid() {
            return State(Point::getInvalid(), GDIP_NAN);
        }

        inline bool isValid() const {
            return (!std::isnan(ang)) && point.isValid();
        }

        inline Point getPoint() const { return point; }

        inline double getAngle() const { return ang; }

        inline double squared_distance(const State &p) const {
            return point.distanceSquared(p.point);
        }

        inline State reverse() const{
            State st = *this;
            st.ang = angleToLeft(0, ang + M_PI);
            return st;
        }

    };

    std::ostream &operator<<(std::ostream &os, const State &d);

}
