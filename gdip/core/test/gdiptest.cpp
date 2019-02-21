/**
* @file circletest.h
* @author Petr Vana
* @brief Test of the optimal solution for the Generalized Dubins Interval Problem (GDIP)
*/

#include "gtest/gtest.h"
#include "opendubins/dubins.h"

using namespace opendubins;
using namespace std;

namespace gdiptest {

    const int COUNT = 2000;

    int typeCount[200] = {};

    void check2(AngleInterval &interval1, AngleInterval &interval2, Dubins &d1, Dubins &d2) {
        EXPECT_TRUE(d1.check()) << "incorrect original maneuver";
        EXPECT_TRUE(d2.check()) << "incorrect newly created maneuver form intervals " << endl << d2;
        if(!d2.check()){
            cout << "Debug point here" << endl;
        }

        bool in1 = interval1.inIntervalWithTollerance(d2.start.ang, TOLERANCE);
        bool in2 = interval2.inIntervalWithTollerance(d2.end.ang, TOLERANCE);
        EXPECT_TRUE(in1) << "out of first interval";
        EXPECT_TRUE(in2) << "out of second interval";
        EXPECT_GE(d1.length + 10 * TOLERANCE, d2.length) << "DubinsInterval " << d2.getType() << " is longer than original maneuver " << d1.getType();

        if (!in1) {
            cout << "IN1 " << d2.start.ang << " is not in <" << interval1.getRight() << ", " << interval1.getLeft() <<
            endl;
        }
        if (!in2) {
            cout << "IN2 " << d2.end.ang << " is not in <" << interval2.getRight() << ", " << interval2.getLeft() <<
            endl;
        }

        if (d1.length + TOLERANCE < d2.length) {
            std::cout << "First dubins 111111111111 " << std::endl << d1 << std::endl;
            std::cout << "Second dubins 222222222222 " << std::endl << d2 << std::endl;
        }

        typeCount[(int) d2.getType()]++;
    }

    bool checkGDIP(AngleInterval &interval1, AngleInterval &interval2, Dubins &d1, Dubins &d2, double diff2) {
        bool ret = true;

        EXPECT_TRUE(d1.check()) << "incorrect original maneuver";
        EXPECT_TRUE(d2.check()) << "incorrect newly created maneuver form intervals";

        bool in1 = interval1.inIntervalWithTollerance(d2.start.ang, TOLERANCE);
        bool in2 = interval2.inIntervalWithTollerance(d2.end.ang, TOLERANCE);
        EXPECT_TRUE(in1) << "out of first interval";
        EXPECT_TRUE(in2) << "out of second interval";
        EXPECT_GE(d1.length + 1 * TOLERANCE, d2.length) << "DubinsInterval is longer than original maneuver";

        if (!in1) {
            cout << "IN1 " << d2.start.ang << " is not in <" << interval1.getRight() << ", " << interval1.getLeft() <<
            endl;
        }
        if (!in2) {
            cout << "IN2 " << d2.end.ang << " is not in <" << interval2.getRight() << ", " << interval2.getLeft() <<
            endl;
        }

        if (d1.length + TOLERANCE < d2.length) {
            std::cout << "First dubins 111111111111 " << std::endl << d1 << std::endl;
            std::cout << "Second dubins 222222222222 " << std::endl << d2 << std::endl;
            ret = false;
        }

        EXPECT_GE(TOLERANCE, interval1.point.distance(d2.start.point)) << "Wrong start " << interval1.point << endl << d2;
        EXPECT_GE(diff2 + TOLERANCE, interval2.point.distance(d2.end.point)) << "Wrong end " << interval2.point << endl << d2;

        typeCount[(int) d2.getType()]++;

        return ret;
    }

    Dubins checkUsingIntervals2(Dubins &d1, double a1, double b1, double a2, double b2) {
        double a = myRandom() * a1;
        double b = myRandom() * b1;

        AngleInterval interval1 = AngleInterval(d1.start.point, d1.start.getAngle() - a, b + a);

        a = myRandom() * a2;
        b = myRandom() * b2;

        AngleInterval interval2 = AngleInterval(d1.end.point, d1.end.getAngle() - a, b + a);

        Dubins d2(interval1, interval2, d1.radius, 0, 0);
        check2(interval1, interval2, d1, d2);

        return d2;
    }

    double getRadius2() {
        return 0.1 + 3 * myRandom();
    }

// OLD TESTS FROM DIP ///////////////////////////////////////////////////////////////////

    TEST (GDIP, ZeroInterval) {
        State p1 = State(Point(0, 0), 0);
        State p2 = State(Point(0, 0), 0);

        for (int i = 0; i < COUNT; ++i) {
            p1.random(100);
            p2.random(100);
            Dubins d1(p1, p2, getRadius2());

            AngleInterval interval1 = AngleInterval(p1.point, p1.ang, 0);
            AngleInterval interval2 = AngleInterval(p2.point, p2.ang, 0);
            Dubins d2(interval1, interval2, d1.radius, 0, 0);

            check2(interval1, interval2, d1, d2);

            Dubins d3(interval1, interval2, d1.radius, 0, 0);
        }
    }

    TEST (GDIP, S) {
        State p1 = State(Point(0, 0), 0);
        for (
                int i = 0;
                i < COUNT;
                ++i) {
            p1.random(100);
            Dubins d1(p1, false, 0, myRandom() * 10, 0, getRadius2());
            Dubins d2 = checkUsingIntervals2(d1, 1, 1, 1, 1);

            EXPECT_LE(fabs(d1.length - d2.length), TOLERANCE) << "Different length of a straight line."
                                << endl << d1 << endl << d2;
        }
    }

    TEST (GDIP, RS) {
        State p1 = State(Point(0, 0), 0);
        for (
                int i = 0;
                i < COUNT;
                ++i) {
            p1.random(100);
            Dubins d1(p1, false, -myRandom() * 7, myRandom() * 10, 0, getRadius2());
            checkUsingIntervals2(d1, 0, 1, 1, 1);
        }
    }

    TEST (GDIP, LS) {
        State p1 = State(Point(0, 0), 0);
        for (
                int i = 0;
                i < COUNT;
                ++i) {
            p1.random(100);
            Dubins d1(p1, false, myRandom() * 7, myRandom() * 10, 0, getRadius2());
            checkUsingIntervals2(d1, 1, 0, 1, 1);
        }
    }

    TEST (GDIP, SR) {
        State p1 = State(Point(0, 0), 0);
        for (
                int i = 0;
                i < COUNT;
                ++i) {
            p1.random(100);
            Dubins d1(p1, false, 0, myRandom() * 10, -myRandom() * 7, getRadius2());
            checkUsingIntervals2(d1, 1, 1, 1, 0);
        }
    }

    TEST (GDIP, SL) {
        State p1 = State(Point(0, 0), 0);
        for (
                int i = 0;
                i < COUNT;
                ++i) {
            p1.random(100);
            Dubins d1(p1, false, 0, myRandom() * 10, myRandom() * 7, getRadius2());
            checkUsingIntervals2(d1, 1, 1, 0, 1);
        }
    }

    TEST (GDIP, Lp) {
        State p1 = State(Point(0, 0), 0);
        for (
                int i = 0;
                i < COUNT;
                ++i) {
            p1.random(100);
            Dubins d1(p1, false, M_PI + myRandom() * M_PI, 0, 0, getRadius2());
            checkUsingIntervals2(d1, 0.1, 0.1, 0.1, 0.1);
        }
    }

    TEST (GDIP, Rp) {
        State p1 = State(Point(0, 0), 0);
        for (int i = 0; i < COUNT;  ++i) {
            p1.random(100);
            Dubins d1(p1, false, -M_PI - myRandom() * M_PI, 0, 0, getRadius2());
            checkUsingIntervals2(d1, 0.1, 0.1, 0.1, 0.1);
        }
    }

    TEST (GDIP, LRp) {
        State p1 = State(Point(0, 0), 0);
        for (int i = 0; i < COUNT; ++i) {
            p1.random(100);

            const double TOL_ANG = 0.05;
            double turn1 = myRandom() * M_PI * (1 - TOL_ANG);
            double turn2 = -M_PI * (1 + TOL_ANG) - myRandom() * M_PI * (1 - 2 * TOL_ANG);

            Dubins d1(p1, false, turn1, 0, turn2, getRadius2());
            checkUsingIntervals2(d1, 0.1, 0, 0.1, 0.1);
        }
    }

    TEST (GDIP, RLp) {
        State p1 = State(Point(0, 0), 0);
        for (
                int i = 0;
                i < COUNT;
                ++i) {
            p1.random(100);

            const double TOL_ANG = 0.05;
            double turn1 = -myRandom() * M_PI * (1 - TOL_ANG);
            double turn2 = +M_PI * (1 + TOL_ANG) + myRandom() * M_PI * (1 - 2 * TOL_ANG);

            Dubins d1(p1, false, turn1, 0, turn2, getRadius2());
            checkUsingIntervals2(d1, 0, 0.1, 0.1, 0.1);
        }
    }

    TEST (GDIP, LpR) {
        State p1 = State(Point(0, 0), 0);
        for (
                int i = 0;
                i < COUNT;
                ++i) {
            p1.random(100);

            const double TOL_ANG = 0.05;
            double turn1 = M_PI * (1 + TOL_ANG) + myRandom() * M_PI * (1 - 2 * TOL_ANG);
            double turn2 = -myRandom() * M_PI * (1 - TOL_ANG);

            Dubins d1(p1, false, turn1, 0, turn2, getRadius2());
            checkUsingIntervals2(d1, 0.1, 0.1, 0.1, 0);
        }
    }

    TEST (GDIP, RpL) {
        State p1 = State(Point(0, 0), 0);
        for (
                int i = 0;
                i < COUNT;
                ++i) {
            p1.random(100);

            const double TOL_ANG = 0.05;
            double turn1 = -M_PI * (1 + TOL_ANG) - myRandom() * M_PI * (1 - 2 * TOL_ANG);
            double turn2 = myRandom() * M_PI * (1 - TOL_ANG);

            Dubins d1(p1, false, turn1, 0, turn2, getRadius2());
            checkUsingIntervals2(d1, 0.1, 0.1, 0, 0.1);
        }
    }

    TEST (GDIP, Small) {
        State p1 = State(Point(0, 0), 0);
        State p2 = State(Point(0, 0), 0);
        for (
                int i = 0;
                i < COUNT;
                ++i) {
            p1.random(5);
            p2.random(5);
            Dubins d1(p1, p2, getRadius2());
            checkUsingIntervals2(d1, 0.1, 0.1, 0.1, 0.1);
        }
    }

    TEST (GDIP, Big) {
        State p1 = State(Point(0, 0), 0);
        State p2 = State(Point(0, 0), 0);
        for (
                int i = 0;
                i < COUNT;
                ++i) {
            p1.random(5);
            p2.random(5);
            Dubins d1(p1, p2, getRadius2());
            checkUsingIntervals2(d1, 1, 1, 1, 1);
        }
    }

    TEST (GDIP, Random) {
        State p1 = State(Point(0, 0), 0);
        for (
                int i = 0;
                i < COUNT;
                ++i) {
            p1.random(5);
            Dubins d1;

            double radius = getRadius2();

            double turn1 = 0;
            double turn2 = 0;

            if ( myRandom() < 0.5) {
                double center = myRandom() * radius;
                d1 = Dubins(p1, false, turn1, center, turn2, radius);
            } else {
                double centerTurn = (1 + myRandom()) * M_PI;
                d1 = Dubins(p1, true, turn1, centerTurn, turn2, radius);
            }
            checkUsingIntervals2(d1, 0.1, 0.1, 0.1, 0.1);
        }
    }

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
// NEW TESTS SPECIALLY FOR GDIP /////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


    Dubins checkUsingIntervals_GDIP(Dubins &d1, double a1, double b1, double a2, double b2, bool random = false, double diff2 = 0.1) {
        diff2 *= myRandom();

        auto dir2 = Vector(myRandom() * 2 * M_PI) * diff2 * (1 - TOLERANCE);
        if(random){
            dir2 *= myRandom();
        }

        double a = myRandom() * a1;
        double b = myRandom() * b1;

        AngleInterval interval1 = AngleInterval(d1.start.point, d1.start.getAngle() - a, b + a);

        a = myRandom() * a2;
        b = myRandom() * b2;

        AngleInterval interval2 = AngleInterval(d1.end.point + dir2, d1.end.getAngle() - a, b + a);

        Dubins d2(interval1, interval2, d1.radius, diff2);
        bool ok = checkGDIP(interval1, interval2, d1, d2, diff2);

        if(!ok){
            cout << "Test is not ok " << endl << "( " << a1 << " " << a2 << " " << b1 << " " << b2 << " " << diff2 << " )" << endl;
        }

        return d2;
    }

    TEST (GDIP, S_GDIP) {
        State p1 = State(Point(0, 0), 0);
        for (int i = 0; i < COUNT; ++i) {
            p1.random(100);
            Dubins d1(p1, false, 0, myRandom() * 10, 0, getRadius2());
            checkUsingIntervals_GDIP(d1, 1, 1, 1, 1);
        }
    }

    void testCpC(bool isLeft, bool basic, bool reverse = false) {
        int sgn = isLeft ? 1 : -1;

        State p1 = State(Point(0, 0), 0);
        for (int i = 0; i < COUNT; ++i) {
            p1.random(100);

            const double TOL_ANG = 0.05;
            double turn1 = myRandom() * M_PI * (1 - TOL_ANG);
            double turn2 = -M_PI * (1 + TOL_ANG) - myRandom() * M_PI * (1 - 2 * TOL_ANG);

            if(reverse){
                std::swap(turn1, turn2);
                sgn = -1;
            }

            Dubins d1(p1, false, sgn*turn1, 0, sgn*turn2, getRadius2());

            double D = 0.1;
            checkUsingIntervals_GDIP(d1, 0, D, D, D, true);
            checkUsingIntervals_GDIP(d1, D, 0, D, D, true);
            checkUsingIntervals_GDIP(d1, D, D, 0, D, true);
            checkUsingIntervals_GDIP(d1, D, D, D, 0, true);

        }
    }

#define TEST_CpC(name, isLeft, reverse) \
    TEST (GDIP, name) { \
        testCpC(isLeft, false, reverse); \
    }


    TEST_CpC(LpP_GDIP, true, true);
    TEST_CpC(RpR_GDIP, false, true);
    TEST_CpC(LRp_GDIP, true, false);
    TEST_CpC(RLp_GDIP, false, false);

    void testCS(bool isLeft, bool basic, bool straight = false, bool longer = false) {
        int sgn = isLeft ? 1 : -1;

        double s = straight ? myRandom() * 0.01 : 0;

        State p1 = State(Point(0, 0), 0);
        for (int i = 0; i < COUNT; ++i) {
            p1.random(100);
            Dubins d1(p1, false, sgn * myRandom() * 3, s, 0, getRadius2());
            if(basic) {
                checkUsingIntervals_GDIP(d1, 0, 0, 0, 0);
            } else {
                checkUsingIntervals_GDIP(d1, 0, 0, 0, 0, true);
                checkUsingIntervals_GDIP(d1, 0, 0, 0, 0, true);
            }
        }
    }

#define TEST_C(name, isLeft) \
    TEST (GDIP, name) { \
        testCS(isLeft, false); \
    }

#define TEST_C_BASIC(name, isLeft) \
    TEST (GDIP, name) { \
        testCS(isLeft, true); \
    }

#define TEST_CS(name, isLeft, straight, longer) \
    TEST (GDIP, name) { \
        testCS(isLeft, false, straight, longer); \
    }

#define TEST_CS_BASIC(name, isLeft, straight, longer) \
    TEST (GDIP, name) { \
        testCS(isLeft, true, straight, longer); \
    }


    TEST_C(L_GDIP_Random, true);
    TEST_C(R_GDIP_Random, false);

    TEST_C_BASIC(L_GDIP_Basic, true);
    TEST_C_BASIC(R_GDIP_Basic, false);

    TEST_CS(LS_GDIP_Random, true, true, false);
    TEST_CS(RS_GDIP_Random, false, true, false);

    TEST_CS_BASIC(LS_GDIP_Basic, true, true, false);
    TEST_CS_BASIC(RS_GDIP_Basic, false, true, false);

    TEST_CS(LS_GDIP_Long, true, true, true);
    TEST_CS(RS_GDIP_Long, false, true, true);

    TEST_CS_BASIC(LS_GDIP_Basic_Long, true, true, true);
    TEST_CS_BASIC(RS_GDIP_Basic_Long, false, true, true);

    void testSC(bool isLeft, bool basic, bool straight = false, bool longer = false) {
        int sgn = isLeft ? 1 : -1;

        double s = straight ? myRandom() * 0.01 : 0;

        State p1 = State(Point(0, 0), 0);
        for (int i = 0; i < COUNT; ++i) {
            p1.random(100);
            Dubins d1(p1, false, 0, s, sgn * myRandom() * 3, getRadius2());
            if(basic) {
                checkUsingIntervals_GDIP(d1, 0, 0, 0, 0);
            } else {
                checkUsingIntervals_GDIP(d1, 0, 0, 0, 0, true);
            }
        }
    }

#define TEST_CB(name, isLeft) \
    TEST (GDIP, name) { \
        testSC(isLeft, false); \
    }

#define TEST_CB_BASIC(name, isLeft) \
    TEST (GDIP, name) { \
        testSC(isLeft, true); \
    }

#define TEST_SC(name, isLeft, straight, longer) \
    TEST (GDIP, name) { \
        testSC(isLeft, false, straight, longer); \
    }

#define TEST_SC_BASIC(name, isLeft, straight, longer) \
    TEST (GDIP, name) { \
        testSC(isLeft, true, straight, longer); \
    }


    TEST_CB(LB_GDIP_Random, true);
    TEST_CB(RB_GDIP_Random, false);

    TEST_CB_BASIC(LB_GDIP_Basic, true);
    TEST_CB_BASIC(RB_GDIP_Basic, false);

    TEST_SC(SL_GDIP_Random, true, true, false);
    TEST_SC(SR_GDIP_Random, false, true, false);

    TEST_SC_BASIC(SL_GDIP_Basic, true, true, false);
    TEST_SC_BASIC(SR_GDIP_Basic, false, true, false);

    TEST_SC(SL_GDIP_Long, true, true, true);
    TEST_SC(SR_GDIP_Long, false, true, true);

    TEST_SC_BASIC(SL_GDIP_Basic_Long, true, true, true);
    TEST_SC_BASIC(SR_GDIP_Basic_Long, false, true, true);


    void testCSC(bool isLeft, bool basic, bool same, double straight = 0, int type = 0) {
        int sgn1 = isLeft ? 1 : -1;
        int sgn2 = (isLeft==same) ? 1 : -1;

        double s = myRandom() * straight;

        State p1 = State(Point(0, 0), 0);
        for (int i = 0; i < COUNT*1; ++i) {
            p1.random(100);
            double r1, r2;

            if(type == 1){
                r1 = sgn1 * myRandom() * 1;
                r2 = fabs(myRandom() * r2) * sgn2;
            }else if (type == 2){
                r2 = sgn2 * myRandom() * 1;
                r1 = fabs(myRandom() * r2) * sgn1;
            } else{
                r1 = sgn1 * myRandom() * 1;
                r2 = sgn2 * myRandom() * 1;
            }

            Dubins d1(p1, false, r1, s, r2, getRadius2());
            if(basic) {
                checkUsingIntervals_GDIP(d1, 0, 0, 0, 0);
            } else {
                checkUsingIntervals_GDIP(d1, 0, 0, 0, 0, true, 0.01);
                checkUsingIntervals_GDIP(d1, 0, 0, 0, 0, true, 0.1);
                checkUsingIntervals_GDIP(d1, 0, 0, 0, 0, true, 1);
                checkUsingIntervals_GDIP(d1, 0, 0, 0, 0, true, 3);
            }
        }
    }

#define TEST_CC(name, isLeft, same) \
    TEST (GDIP, name) { \
        testCSC(isLeft, false, same); \
    }

#define TEST_CC_BASIC(name, isLeft, same) \
    TEST (GDIP, name) { \
        testCSC(isLeft, true, same); \
    }

#define TEST_CSC_ALL(name, isLeft, false, same, straight) \
    TEST (GDIP, name) { \
        testCSC(isLeft, false, same, straight, 0); \
    }\
    TEST (GDIP, name##_Xx) { \
        testCSC(isLeft, false, same, straight, 1); \
    }\
    TEST (GDIP, name##_xX) { \
        testCSC(isLeft, false, same, straight, 2); \
    }

#define TEST_CSC(name, isLeft, same, straight) \
    TEST_CSC_ALL(name, isLeft, false, same, straight)

#define TEST_CSC_BASIC(name, isLeft, same, straight) \
        TEST_CSC_ALL(name, isLeft, true, same, straight)

#define BOOL_L true
#define BOOL_R false

#define TEST_CSC_BATCH(first, second) \
    TEST_CC_BASIC(first##second##_GDIP_Basic, BOOL_##first, BOOL_##second) \
    TEST_CC(first##second##_GDIP_Random, BOOL_##first, BOOL_##second) \
    TEST_CSC_BASIC(first##S##second##_GDIP_Basic, BOOL_##first, BOOL_##second, 1) \
    TEST_CSC(first##S##second##_GDIP_Random, BOOL_##first, BOOL_##second, 1)

    // SAME SIDE
    TEST_CSC_BATCH(L, L);
    TEST_CSC_BATCH(R, R);

    // DIFF SIDE
    TEST_CSC_BATCH(R, L);
    TEST_CSC_BATCH(L, R);


    TEST (GDIP, AllUsed) {
        for (int t = (int)DType::GDIP_NO; t <= (int)DType::GDIP_RpL; t++) {
            DType typ = static_cast<DType>(t);
            const int value = typeCount[t];
            std::cout << typ << "\t-\t" << value << endl;
            EXPECT_GT(value, 0) << "There is no maneuver with type " << typ;
        }
    }

}