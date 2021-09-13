/**
 * @file dmath.h
 * @author Petr Vana
 * @brief Collection of math function for the OpenDubins library
 */

#pragma once

#include <stdlib.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include <random>
#include <ctime>

namespace opendubins {

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884 /* pi use l for long double */
#endif

#ifndef M_PI_2
#define M_PI_2 (M_PI/2)
#endif

#define GDIP_NAN std::numeric_limits<double>::quiet_NaN()

    const double TOLERANCE = 1e-5;

    static unsigned _dmath_current_seed = 42;

    static std::default_random_engine opendubins_random_generator;

    inline double myRandom() {
        //return rand() / (myFloat)RAND_MAX;
        //return random() / (double) RAND_MAX;
        return std::uniform_real_distribution<double>(0,1)(opendubins_random_generator);
    }

    inline void setSeed(unsigned seed) {
        /*if (seed == 42) {
            struct timespec ts; // C11 standard
            timespec_get(&ts, TIME_UTC);
            seed = (ts.tv_nsec);
        }*/
        _dmath_current_seed = seed;
        opendubins_random_generator = std::default_random_engine(seed);
    }

    inline int myIntRandom(const int size) {
        //return rand() % size;
        //return random() % size;
        return std::uniform_int_distribution<int>(0,size-1)(opendubins_random_generator);
    }

    template<typename T>
    constexpr int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    inline double angleToLeft(const double &ang1, const double &ang2) {
        double ret = ang2 - ang1;

        while (ret > 2 * M_PI) {
            ret -= 2 * M_PI;
        }
        while (ret < 0) {
            ret += 2 * M_PI;
        }
        return ret;
    }

    inline double angleToRight(const double &ang1, const double &ang2) {
        double ret = ang2 - ang1;

        while (ret < -2 * M_PI) {
            ret += 2 * M_PI;
        }
        while (ret > 0) {
            ret -= 2 * M_PI;
        }

        return ret;
    }

    inline double angleToSide(const double &ang1, const double &ang2, const bool& isLeft){
        return isLeft ? angleToLeft(ang1, ang2) : angleToRight(ang1, ang2);
    }

    inline double checkToleranceLeft(double turn) {
        return (turn > 2 * M_PI - TOLERANCE && turn < 2 * M_PI + TOLERANCE) ? 0 : turn;
    }

    inline double checkToleranceRight(double turn) {
        return (turn < -2 * M_PI + TOLERANCE && turn > -2 * M_PI - TOLERANCE) ? 0 : turn;
    }

    inline void randomOrder(std::vector<int> &order, int size) {
        order.clear();
        for (int i = 0; i < size; i++)
            order.push_back(i);
        std::random_shuffle(order.begin(), order.end(), myIntRandom);
    }

    constexpr int modulo(int a, int b) {
        return (a + b) % b;
    }

}
