/**
 * @file circle.cpp
 * @author Petr Vana
 * @brief Circle class used for calculating intersections, etc.
 */

#include "circle.h"

using namespace std;

namespace opendubins {

    Circle::Circle(const Point &center, const double &radius) {
        this->center = center;
        this->radius = radius;
    }

    Circle::~Circle() { }

    bool Circle::pointInCircle(const Point& p) const {
        return center.distanceSquared(p) <= radius * radius;
    }

    Point Circle::getCenter() const {
        return center;
    }

    double Circle::perimeterLength() const {
        return 2 * M_PI * radius;
    }


    tuple<int, Point, Point> Circle::halfLineIntersections(const State &pos) const {
        int count = 0;
        Point ps[2];

        // calculate two points of intersection
        Vector dir = pos.getNormalizedDirection(); // (already normalized)
        Vector normal = dir.left();
        double distance = normal.dotProduct(center - pos.point);

        // vector to closest point on line from center of arc
        Vector vDistance = -distance * normal;

        if (distance < radius) {
            double tangentAngle = vDistance.getAngle();
            double diffAngle = acos(fabs(distance) / radius);

            double ang1 = tangentAngle + diffAngle;
            double ang2 = tangentAngle - diffAngle;

            auto p1 = center + Vector(ang1) * radius;
            auto p2 = center + Vector(ang2) * radius;

            if (dir.dotProduct(p1 - pos.point) > 0) {
                ps[count++] = p1;
            }

            if (dir.dotProduct(p2 - pos.point) > 0) {
                ps[count++] = p2;
            }
        }

        return tuple<int, Point, Point>(count, ps[0], ps[1]);
    }

    Point Circle::halfLineIntersection(const State &pos) const {
        tuple<int, Point, Point> ints = halfLineIntersections(pos);
        int count = get<0>(ints);

        if (count > 1) {
            auto len1 = pos.point.distance(get<1>(ints));
            if (count == 1) {
                return get<1>(ints);
            } else {
                auto len2 = pos.point.distance(get<2>(ints));
                if (len1 < len2) {
                    return get<1>(ints);
                } else {
                    return get<2>(ints);
                }
            }
        }
        return Point::getInvalid();
    }

    tuple<State, double> Circle::firstIntersection(const Arc& arc) const{
        auto arcCenter = arc.getCenter();
        Vector diff = center - arcCenter;

        double diffLen = diff.length();

        // alpha = acos((b*b + c*c - a*a)/(2*b*c));
        double a = radius;
        double b = arc.radius;
        double c = diffLen;

        double arccos = (b*b + c*c - a*a)/(2*b*c);
        if(arccos >= -1 && arccos <= 1){
            double alpha = acos(arccos);

            double orient = arc.getAngle() > 0 ? M_PI_2 : -M_PI_2;

            double dir1 = diff.getAngle() + alpha + orient;
            double dir2 = diff.getAngle() - alpha + orient;

            double stAng = arc.getStart().ang;

            double turn1 = arc.getAngle() > 0 ? angleToLeft(stAng, dir1)
                                              : angleToRight(stAng, dir1);
            double turn2 = arc.getAngle() > 0 ? angleToLeft(stAng, dir2)
                                              : angleToRight(stAng, dir2);

            if(fabs(turn1) < fabs(turn2)){
                return tuple<State, double>(arc.getState(fabs(turn1 * arc.radius)), fabs(turn1 * arc.radius));
            }else{
                return tuple<State, double>(arc.getState(fabs(turn2 * arc.radius)), fabs(turn2 * arc.radius));
            }

        }

        return tuple<State, double>(State::getInvalid(), -1);
    }

    AngleInterval Circle::getIntersectionAngleInterval(const Circle& other) const {
        AngleInterval interval = AngleInterval();

        Vector diff = other.center - center;
        double diffLen = diff.length();

        // alpha = acos((b*b + c*c - a*a)/(2*b*c));
        double a = other.radius;
        double b = radius;
        double c = diffLen;

        double arccos = (b*b + c*c - a*a)/(2*b*c);
        if(arccos >= -1 && arccos <= 1){
            double alpha = acos(arccos);
            double diffAngle = diff.getAngle();
            return AngleInterval(center, diffAngle - alpha, 2*alpha);
        }

        // this circle is completely in the other one
        if(diffLen < other.radius && radius < other.radius){
            return AngleInterval(center, 0, 2*M_PI);
        }

        return interval;
    }

    vector<AngleInterval> Circle::getIntersectionAngleIntervals(const vector<Circle> circles) const {
        vector<AngleInterval> intersections;

        for(auto & c : circles){
            auto inter = getIntersectionAngleInterval(c);
            if(inter.isValid()){
                intersections.push_back(inter);
            }
        }

        return AngleInterval::mergeIntervals(intersections);
    }

    Vector Circle::shortestVectorInSector(const AngleInterval& sector) const {
        // zero distance
        if(pointInCircle(sector.point)){
            return Vector(0,0);
        }

        // directly to the circle
        auto diff = center - sector.point;
        if(sector.inInterval(diff.getAngle())){
            auto diff2 = diff.normalize() * -radius;
            return (center + diff2) - sector.point;
        }

        // at the edge of the sector
        Point p1 = halfLineIntersection(sector.getLeftState());
        Point p2 = halfLineIntersection(sector.getRightState());

        Point ret = Point::getInvalid();

        double len = numeric_limits<double>::max();
        if(p1.isValid()){
            double nLen = p1.distance(sector.point);
            if(nLen < len){
                len = nLen;
                ret = p1;
            }
        }
        if(p2.isValid()){
            double nLen = p2.distance(sector.point);
            if(nLen < len){
                //len = nLen; // not used anymore
                ret = p2;
            }
        }

        if(ret.isValid()){
            return ret - sector.point;
        }else{
            return Vector::getInvalid();
        }
    }

    Point Circle::closestPointInSector(const AngleInterval& sector) const {
        return sector.point + shortestVectorInSector(sector);
    }

}
