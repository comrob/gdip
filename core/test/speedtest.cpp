/**
* @file circletest.h
* @author Petr Vana
* @brief Compare speed of original Dubins maneuver, DIP, and GDIP implementation
*/

#include "gtest/gtest.h"
#include "opendubins/dubins.h"

using namespace opendubins;
using namespace std;

namespace speedtest {

    const int COUNT = 1000;
    const int COUNT2 = 1000;

    double box = 10;

    TEST (SPEED, GDIP) {
        double sum = 0;
        for(int i = 0; i < COUNT; i++) {
            double radius = 0;

            double a1 = 2 * M_PI * myRandom();
            double b1 = 2 * M_PI * myRandom();
            double a2 = 2 * M_PI * myRandom();
            double b2 = 2 * M_PI * myRandom();

            Point p1(box * myRandom(), box * myRandom());
            Point p2(box * myRandom(), box * myRandom());

            AngleInterval interval1 = AngleInterval(p1, -a1, b1 + a1);
            AngleInterval interval2 = AngleInterval(p2, -a2, b2 + a2);

            double diff1 = myRandom();
            double diff2 = myRandom();

            for(int j = 0; j < COUNT2; j++) {
                Dubins d2(interval1, interval2, radius, diff1, diff2);
                sum += d2.length;
            }
        }
        if(sum == 0) std::cout << "Error" << std::endl;
    }

    TEST (SPEED, DIP) {
        double sum = 0;
        for(int i = 0; i < COUNT; i++) {
            double radius = 0;

            double a1 = 2 * M_PI * myRandom();
            double b1 = 2 * M_PI * myRandom();
            double a2 = 2 * M_PI * myRandom();
            double b2 = 2 * M_PI * myRandom();

            Point p1(box * myRandom(), box * myRandom());
            Point p2(box * myRandom(), box * myRandom());

            AngleInterval interval1 = AngleInterval(p1, -a1, b1 + a1);
            AngleInterval interval2 = AngleInterval(p2, -a2, b2 + a2);

            for(int j = 0; j < COUNT2; j++) {
                Dubins d2(interval1, interval2, radius);
                sum += d2.length;
            }
        }
        if(sum == 0) std::cout << "Error" << std::endl;
    }

    TEST (SPEED, Dubins) {
        double sum = 0;
        for(int i = 0; i < COUNT; i++) {
            double radius = 0;

            double a1 = 2 * M_PI * myRandom();
            double b1 = 2 * M_PI * myRandom();
            double a2 = 2 * M_PI * myRandom();
            double b2 = 2 * M_PI * myRandom();

            Point p1(box * myRandom(), box * myRandom());
            Point p2(box * myRandom(), box * myRandom());

            AngleInterval interval1 = AngleInterval(p1, -a1, b1 + a1);
            AngleInterval interval2 = AngleInterval(p2, -a2, b2 + a2);

            for(int j = 0; j < COUNT2; j++) {
                Dubins d2(interval1.getLeftState(), interval2.getLeftState(), radius);
                sum += d2.length;
            }
        }
        if(sum == 0) std::cout << "Error" << std::endl;
    }

}