/**
 * @file line.cpp
 * @author Petr Vana
 * @brief Line is a basic segment in Dubins maneuver
 */

#include "line.h"

namespace opendubins {

    Line::Line(const Point &np1, const Point &np2) {
        p1 = np1;
        p2 = np2;
    }

    Line::Line(State pos, double len) {
        p1 = pos.point;
        p2 = pos.point + pos.getNormalizedDirection() * len;
    }

    State Line::getState(double len) const {
        Vector dir = getDirection();
        Point p = p1 + dir.normalize() * len;
        if(dir.dx == 0 && dir.dy == 0){
            return State(p1, 0);
        }
        return State(p, dir.getAngle());
    }

    StateAtDistance Line::getClosestStateAndDistance(const Point &p) const {
        StateAtDistance ret;

        auto len = getLength();
        if (len < TOLERANCE) {
            ret.state = State(p1, 0);
            ret.distance = 0;
        } else {
            auto dir = (p2 - p1).normalize();

            auto usedLen = (p - p1).dotProduct(dir);
            if (usedLen < 0) {
                ret.state = getStart();
                ret.distance = 0;
            } else if (usedLen > len) {
                ret.state = getEnd();
                ret.distance = getLength();
            } else {
                ret.state = getState(usedLen);
                ret.distance = usedLen;
            }
        }

        return ret;
    }

    bool Line::intersect(Line line) const {
        // normales to directions
        Vector norm1 = this->getDirection().left().normalize();
        Vector norm2 = line.getDirection().left().normalize();

        // end point distances from line1
        double d11 = (line.p1 - this->p1).dotProduct(norm1);
        double d12 = (line.p2 - this->p1).dotProduct(norm1);

        // end point distances from line2
        double d21 = (this->p1 - line.p1).dotProduct(norm2);
        double d22 = (this->p2 - line.p1).dotProduct(norm2);

        // end points of first line are at opposite sides of second line
        if (d11 * d12 < 0 && d21 * d22 < 0) {
            return true;
        }

        return false;
    }

    StateAtDistance Line::intersectionPoint(Line line) const {

        if (intersect(line)) {
            Vector dir2 = line.getDirection();
            Vector normal2 = dir2.normalize().left();

            double dist = normal2.dotProduct(p1 - line.p1);
            double speed = -normal2.dotProduct(getDirection().normalize());

            if (dist * speed > 0) {
                dist = dist / speed;

                Point inter = p1 + getDirection().normalize() * dist;
                return StateAtDistance(State(inter, getDirection().getAngle()), dist);
                // TODO
                //return Intersection(Position(Point(0,0), 0), 0);
            }
        }

        return StateAtDistance(State(), 0);
    }

}

