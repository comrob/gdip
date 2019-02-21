/**
* @file circletest.h
* @author Petr Vana
* @brief Test implementation for intersections with cirecle
*/

#include "gtest/gtest.h"
#include "opendubins/arc.h"
#include "opendubins/circle.h"

using namespace std;
using namespace opendubins;

namespace circletest {

    const int COUNT = 1000;

    void check(Circle& c, Arc& arc){
        State pr;
        double len;
        tie(pr, len) = c.firstIntersection(arc);

        Point pr2 = arc.getState(len).point;

        EXPECT_TRUE(pr.point.distance(pr2) < TOLERANCE) << "To far";
        EXPECT_TRUE(fabs(c.center.distance(pr.point) - c.radius) < c.radius * TOLERANCE);
    }


    TEST (CircleTest, ArcIntersectionRight) {
        Circle c(Point(0,0), 0.9);
        Arc arc(State(-1,-1,M_PI), -2*M_PI, 1);

        check(c, arc);
    }

    TEST (CircleTest, ArcIntersectionLeft) {
        Circle c(Point(0,0), 1);
        Arc arc(State(1,1,M_PI), 2*M_PI, 1.1);

        check(c, arc);
    }

    TEST (CircleTest, ArcIntersectionRandom) {

        for(int i = 0; i < COUNT; ++i) {
            double radius = 0.1 +  2*myRandom();
            Circle c(Point(0, 0), radius);
            State s;
            s.random(2);

            auto a = myRandom();
            auto angle = a > 0.5 ? 2*M_PI : -2*M_PI;
            Arc arc(s, angle, 2*myRandom());

            if (c.center.distance(arc.getCenter()) < c.radius + arc.radius) {
                if (c.center.distance(arc.getCenter()) > fabs(c.radius - arc.radius)) {
                    check(c, arc);
                }
            }
        }
    }

    TEST (CircleTest, CircleIntersectionRandom) {

        const int COUNT_INTER = 10;

        for(int i = 0; i < COUNT / COUNT_INTER; ++i) {
            double radius1 =  2*myRandom();
            double radius2 =  2*myRandom();

            State s1, s2;
            s1.random(2);
            s2.random(2);

            Circle c1(s1.point, radius1);
            Circle c2(s2.point, radius2);

            auto intersect = c1.getIntersectionAngleInterval(c2);

            if(intersect.isValid()) {
                for (int j = 0; j < COUNT_INTER; ++j) {
                    double angle = 2 * M_PI * myRandom();
                    Point randomPoint = c1.center + Vector(angle) * c1.radius;

                    if(intersect.inInterval(angle)){
                        EXPECT_LE(randomPoint.distance(c2.center), c2.radius + TOLERANCE) << "FP - Out of the circle";
                    }else{
                        EXPECT_GE(randomPoint.distance(c2.center), c2.radius - TOLERANCE) << "FN - In the circle";
                    }
                }
            }else{
                double distance = s1.point.distance(s2.point);
                if(distance < radius1 + radius2 - TOLERANCE){
                    if(radius1 < radius2 || distance > radius1 - radius2) {
                        EXPECT_TRUE(false) << "There should be some intersection "
                                           << endl << s1.point
                                           << endl << s2.point
                                           << endl << "radii: " << radius1 << ", " << radius2
                                           << endl << distance << endl;
                    }
                }
            }
        }
    }

    TEST (CircleTest, SectorIntersectionValid) {
        for(int i = 0; i < COUNT; ++i) {
            double radius1 =  2*myRandom();

            State s1, s2;
            s1.random(2);
            s2.random(2);

            Circle circle(s1.point, radius1);
            AngleInterval sector(s2.point, s2.ang, 0.1);

            auto vec = circle.shortestVectorInSector(sector);
            auto intersect = sector.point + vec;

            if (intersect.isValid()) {
                Vector diff = intersect - sector.point;
                if(diff.length() > TOLERANCE) {
                    EXPECT_TRUE(sector.inIntervalWithTollerance(diff.getAngle(), TOLERANCE))
                                        << "Point is out of the sector";
                } else {
                    EXPECT_LE(diff.length(), std::max(circle.center.distance(sector.point) - circle.radius + TOLERANCE, TOLERANCE));
                }
                EXPECT_LE(intersect.distance(circle.center), circle.radius + TOLERANCE) << "Point is out of the circle";
            }
        }
    }

    TEST (CircleTest, SectorIntersectionRandom) {

        const int COUNT_INTER = 10;
        const double M = numeric_limits<double>::max();

        for(int i = 0; i < COUNT / COUNT_INTER; ++i) {
            double radius1 =  2*myRandom();

            State s1, s2;
            s1.random(2);
            s2.random(2);

            Circle circle(s1.point, radius1);
            AngleInterval sector(s2.point, s2.ang, 0.1);

            auto vec = circle.shortestVectorInSector(sector);
            auto intersect = sector.point + vec;

            double len = M;

            for(int j = 0; j < COUNT_INTER; j++){
                double ang = myRandom() * 2 * M_PI;
                Point np = circle.center + Vector(ang) * circle.radius;
                if(sector.inInterval((np - sector.point).getAngle())){
                    len = min(len, np.distance(sector.point));
                }
            }

            if (intersect.isValid()) {
                EXPECT_LE(intersect.distance(sector.point), len + TOLERANCE) << "There is a closer point";
            }else{
                EXPECT_EQ(len, M) << "Some intersection should be found";
            }
        }
    }


}
