/**
 * @file angleinterval.h
 * @author Petr Vana
 * @brief Interval of heading angles utilized in both DIP and GDIP formulation
 */

#pragma once

#include <algorithm>

#include "dmath.h"
#include "state.h"

namespace opendubins {

    struct AngleInterval {

        // point to which the angle interval belongs
        Point point;

        // right angle from the interval
        // startAngle \in <0,2*PI)
        double rightDir;

        // diff between left and right angle <0,2*PI>
        // diff = 0    >> single direction
        // diff = 2*PI >> every possible dirrection
        double diff;

        int resolution;

        AngleInterval() : point(Point(NAN, NAN)), rightDir(NAN), diff(NAN) {
            resolution = -1;
        };

        AngleInterval(Point point, double rightDir, double diff) : point(point), rightDir(rightDir) {
            this->diff = angleToLeft(0, diff);
            resolution = -1;
            normalize();
        };

        AngleInterval(State state, double diff) : point(state.point), rightDir(state.ang) {
            this->diff = angleToLeft(0, diff);
            resolution = -1;
            normalize();
        };

        AngleInterval(State state) : point(state.point), rightDir(state.ang), diff(2 * M_PI) {
            resolution = 1;
            normalize();
        };


        AngleInterval(Point point) : point(point), rightDir(0), diff(2 * M_PI) {
            resolution = 1;
            normalize();
        };

        AngleInterval(Point point, double rightDir, double diff, int resolution) : point(point), rightDir(rightDir),
                                                                                   diff(diff),
                                                                                   resolution(resolution) {
            normalize();
        };

        inline double getRight() const {
            return rightDir;
        }

        inline double getLeft() const {
            return rightDir + diff;
        }

        inline State getLeftState() const {
            return State(point, getLeft());
        }

        inline State getRightState() const {
            return State(point, getRight());
        }

        inline Vector getLeftDir() const {
            return Vector(getLeft());
        }

        inline Vector getRightDir() const {
            return Vector(getRight());
        }

        inline double inInterval(double direction) const {
            return angleToLeft(rightDir, direction) <= diff;
        }

        inline double inIntervalWithTollerance(double direction, double tollerance) {
            if (angleToLeft(rightDir, direction) <= diff + tollerance) {
                return true;
            } else {
                if (angleToLeft(rightDir, direction) + tollerance > 2 * M_PI) {
                    return true;
                } else {
                    return false;
                }
            }
        }

        inline AngleInterval splitInterval() {
            diff /= 2;
            resolution *= 2;
            AngleInterval ret(point, getLeft(), diff);
            ret.resolution = resolution;
            return ret;
        }

        inline AngleInterval reverse() const {
            double newAngle = angleToLeft(0, rightDir + M_PI);
            return AngleInterval(point, newAngle, diff, resolution);
        }

        inline bool operator<(const AngleInterval &o) const {
            return rightDir < o.rightDir;
        }

        inline bool operator==(const AngleInterval &o) const {
            return (point == o.point) && (rightDir == o.rightDir) && (diff == o.diff);
        }

        // hash point and angles
        inline size_t hash() const {
            auto h1 = point.hash();
            auto h2 = std::hash<double>()(diff);
            auto h3 = std::hash<double>()(rightDir);
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }

        // hash only angles
        inline size_t angle_hash() const {
            auto h1 = std::hash<double>()(diff);
            auto h2 = std::hash<double>()(rightDir);
            return h1 ^ (h2 << 1);
        }

        inline bool isValid() const {
            return !std::isnan(rightDir);
        }

        inline void normalize() {
            rightDir = angleToLeft(0, rightDir);
        }

        static std::vector<AngleInterval> mergeIntervals(std::vector<AngleInterval> input);
    };

    std::ostream &operator<<(std::ostream &os, const AngleInterval &d);
}
