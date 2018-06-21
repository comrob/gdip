/**
 * @file arc.cpp
 * @author Petr Vana
 * @brief Arc is a basic segment in Dubins maneuver
 */

#include "arc.h"

namespace opendubins {

    State Arc::getState(double len) const {
        return getStateByAngle(len * sgn(angle) / radius);
    }

    StateAtDistance Arc::getClosestStateAndDistance(const Point &p) const {
        StateAtDistance ret;

        // start state
        ret.state = state;
        ret.distance = 0;

        // end state
        auto end = getEnd();
        if (end.point.distance(p) < ret.state.point.distance(p)) {
            ret.state = end;
            ret.distance = getLength();
        }

        // center state
        if (angle > 0) { // left turn
            auto dir = (p - getCenter()).left().getAngle();
            auto turn = angleToLeft(state.ang, dir);
            auto turnLen = turn * radius;
            if (turnLen < getLength()) {
                auto actPos = getState(turnLen);
                if (actPos.point.distance(p) < ret.state.point.distance(p)) {
                    ret.state = actPos;
                    ret.distance = turnLen;
                }
            }
        } else { // right turn
            auto dir = (p - getCenter()).right().getAngle();
            auto turn = angleToRight(state.ang, dir);
            auto turnLen = -turn * radius;
            if (turnLen < getLength()) {
                auto actPos = getState(turnLen);
                if (actPos.point.distance(p) < ret.state.point.distance(p)) {
                    ret.state = actPos;
                    ret.distance = turnLen;
                }
            }
        }

        return ret;
    }

    StateAtDistance Arc::intersectionPoint(const Line &line) const {
        double rad = getLength() + line.getLength();
        if ((state.point - line.p1).lengthSquared() > rad * rad) {
            return StateAtDistance(State(), 0);
        }

        // calculate two points of intersection
        Vector dir = line.getDirection().normalize();
        Vector normal = dir.left();
        double distance = normal.dotProduct(getCenter() - line.p1);

        // vector to closest point on line from center of arc
        Vector vDistance = -distance * normal;

        if (distance > radius) {
            return StateAtDistance(State(), 0);
        }

        double tangentAngle = vDistance.getAngle() + (angle > 0 ? M_PI / 2 : -M_PI / 2);
        double diffAngle = acos(fabs(distance) / radius);

        double ang1 = tangentAngle + diffAngle;
        double ang2 = tangentAngle - diffAngle;

        if (angle > 0) { // left
            double turn1 = angleToLeft(state.ang, ang1);
            double turn2 = angleToLeft(state.ang, ang2);

            double less = std::min(turn1, turn2);
            double more = std::max(turn1, turn2);

            if (less <= angle) {
                State p = getStateByAngle(less);
                double dist = dir.dotProduct(p.point - line.p1);

                if (dist >= 0 && dist < line.getDirection().length()) {
                    return StateAtDistance(p, less * radius);
                }

                if (more <= angle) {
                    p = getStateByAngle(more);
                    dist = dir.dotProduct(p.point - line.p1);

                    if (dist >= 0 && dist < line.getDirection().length()) {
                        return StateAtDistance(p, more * radius);
                    }
                }
            }
        } else {
            double turn1 = angleToRight(state.ang, ang1);
            double turn2 = angleToRight(state.ang, ang2);

            double less = std::max(turn1, turn2);
            double more = std::min(turn1, turn2);

            if (less >= angle) {
                State p = getStateByAngle(less);
                double dist = dir.dotProduct(p.point - line.p1);

                if (dist >= 0 && dist < line.getDirection().length()) {
                    return StateAtDistance(p, less * -radius);
                }

                if (more >= angle) {
                    p = getStateByAngle(more);
                    dist = dir.dotProduct(p.point - line.p1);

                    if (dist >= 0 && dist < line.getDirection().length()) {
                        return StateAtDistance(p, more * -radius);
                    }
                }
            }
        }

        // todo - try to refactor

        return StateAtDistance(State(), 0);
    }

    Point Arc::getCenter() const {
        Point center = state.point;

        Vector toCenter = state.getNormalizedDirection().left();
        toCenter *= radius;
        if (angle < 0) {
            toCenter *= -1;
        }

        center += toCenter;

        return center;
    }

    State Arc::getStateByAngle(double arcAngle) const {
        Point center = getCenter();

        Vector dir = state.getNormalizedDirection() * radius;
        Vector norm = state.point - center;

        double aa = std::fabs(arcAngle);

        center += dir * std::sin(aa);
        center += norm * std::cos(aa);

        return State(center, state.ang + arcAngle);
    }

    State Arc::getEnd() const {
        return getStateByAngle(angle);
    }


}
