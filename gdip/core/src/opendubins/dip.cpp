/**
 * @file dip.cpp
 * @author Petr Vana
 * @brief Implementation of the Dubins Interval Problem (DIP)
 * - shortest path connecting tow states with heading intervals
 */

#include <iomanip>
#include "dubins.h"

namespace opendubins {

    Dubins::Dubins(AngleInterval from, AngleInterval to, double newRadius) {
        radius = newRadius;
        length = std::numeric_limits<double>::max();
        type = DType::Unknown;
        initIntervalProblem(from, to);
    }

// implementation of dubins Interval Problem
// based on article http://arxiv.org/pdf/1507.06980v2.pdf
    void Dubins::initIntervalProblem(const AngleInterval & from, const AngleInterval & to) {
        const Vector dir = to.point - from.point;
        const double dir_length = dir.length();
        const double dir_ang = dir.getAngle();

        ///////////////////////////////
        // Pre-computation

        Vector v1left = from.getLeftDir();
        Vector v1right = from.getRightDir();
        Point c1left = from.point + v1left.left() * radius;
        Point c1right = from.point + v1right.right() * radius;

        Vector v2left = to.getLeftDir();
        Vector v2right = to.getRightDir();
        Point c2left = to.point + v2right.left() * radius;
        Point c2right = to.point + v2left.right() * radius;

        ///////////////////////////////
        // CASE 1

        // Straight line S
        if (from.inInterval(dir_ang) && to.inInterval(dir_ang)) {
            isCCC = false;
            len1 = len3 = 0;
            length = len2 = dir_length;
            start = State(from.point, dir_ang);
            end = State(to.point, dir_ang);
            type = DType::DIP_S;
        }

        ///////
        // + is greater then 1*PI
        // L+, R+, L+R+, R+L+

        // L+, R+
        if (dir_length < 2 * radius) {
            double alpha = std::asin(dir_length / 2 / radius);

            // L+
            {
                double dir1 = dir_ang + M_PI + alpha;
                double dir2 = dir_ang + M_PI - alpha;

                double turn = 2 * (M_PI - alpha);

                double nLen = radius * turn;
                if (nLen < length) {
                    if (from.inInterval(dir1) && to.inInterval(dir2)) {
                        length = nLen;
                        isCCC = false;
                        len1 = turn;
                        len2 = 0;
                        len3 = 0;

                        start = State(from.point, dir1);
                        end = State(to.point, dir2);

                        type = DType::DIP_Lp;
                    }
                }
            }

            // R+
            {
                double dir1 = dir_ang + M_PI - alpha;
                double dir2 = dir_ang + M_PI + alpha;

                double turn = 2 * (-M_PI + alpha);

                double nLen = -radius * turn;
                if (nLen < length) {
                    if (from.inInterval(dir1) && to.inInterval(dir2)) {
                        length = nLen;
                        isCCC = false;
                        len1 = turn;
                        len2 = 0;
                        len3 = 0;

                        start = State(from.point, dir1);
                        end = State(to.point, dir2);

                        type = DType::DIP_Rp;
                    }
                }
            }
        }

#if USE_XpXp
        // R+L+, L+R+
        if (dir_length < 4 * RADIUS) {

            double alpha = asin(dir_length / 4 / RADIUS);

            // R+L+
            {
                double dir1 = dir_ang + M_PI - alpha;
                double dir2 = dir_ang + M_PI + alpha;

                double turn = angleToRight(dir1, dir2);

                double nLen = - 2 * RADIUS * turn;
                if (nLen < length) {
                    if (from.inInterval(dir1) && to.inInterval(dir2)) {
                        length = nLen;
                        isCCC = false;
                        len1 = turn;
                        len2 = 0;
                        len3 = -turn;

                        start = State(from.point, dir1);
                        end = State(to.point, dir2);

                        type = DType::DIP_RpLp;
                    }
                }
            }

            // L+R+
            {
                double dir1 = dir_ang + M_PI + alpha;
                double dir2 = dir_ang + M_PI - alpha;

                double turn = angleToLeft(dir1, dir2);

                double nLen = 2 * RADIUS * turn;
                if (nLen < length) {
                    if (from.inInterval(dir1) && to.inInterval(dir2)) {
                        length = nLen;
                        isCCC = false;
                        len1 = turn;
                        len2 = 0;
                        len3 = -turn;

                        start = State(from.point, dir1);
                        end = State(to.point, dir2);

                        type = DType::DIP_LpRp;
                    }
                }
            }
        }
#endif

        ///////////////////////////////
        // CASE 2
        // on boundaries of angle interval
        // LSR, LSL, LR+L, RSL, RSR, RL+R
        initForDIP(from.getRightState(), to.getRightState(), v1right, v2right);
        initForDIP(from.getRightState(), to.getLeftState(), v1right, v2left);
        initForDIP(from.getLeftState(), to.getRightState(), v1left, v2right);
        initForDIP(from.getLeftState(), to.getLeftState(), v1left, v2left);

        ///////////////////////////////
        // CASE 3 - limited by first interval
        // LS, LR+, RS, RL+
        {
            // vector from the center of the left turn to the goal point
            Vector diff = to.point - c1left;
            double diffAngle = diff.getAngle();
            double len = diff.length();

            // LS
            if (radius <= len) {
                double alpha = std::asin(radius / len);
                double directionS = diffAngle + alpha;
                double turn = angleToLeft(from.getLeft(), directionS);

                State st = from.getLeftState();
                double lenS = (c1left + (from.point - c1left).rotate(turn)).distance(to.point);

                double nLen = lenS + radius * turn;
                if (nLen < length) {
                    if (to.inInterval(directionS)) {
                        length = nLen;
                        isCCC = false;
                        len1 = turn;
                        len2 = lenS;
                        len3 = 0;

                        start = st;
                        end = State(to.point, directionS);

                        type = DType::DIP_LS;
                    }
                }
            }

            // LR+
            if (radius < len && len < 3 * radius) {

                // angle in triangle 2R, R, len
                double alpha = std::acos((3 * radius * radius + len * len) / (4 * radius * len));

                { // first option + alpha
                    // direction between segments
                    double centerDir = diffAngle + M_PI / 2 + alpha;

                    // length of the first turning maneuver (L)
                    double turn1 = angleToLeft(from.getLeft(), centerDir);
                    Point centerPoint = (c1left + (from.point - c1left).rotate(turn1));

                    // L+
                    double dir2 = centerPoint.distance(to.point);
                    if (dir2 < 2 * radius) {
                        double alpha2 = std::asin(dir2 / 2 / radius);

                        double turn2 = -2 * M_PI + 2 * alpha2;
                        double dir22 = from.getLeft() + turn1 + turn2;

                        double nLen = radius * (turn1 - turn2);
                        if (nLen < length) {
                            if (to.inInterval(dir22)) {
                                length = nLen;
                                isCCC = false;
                                len1 = turn1;
                                len2 = 0;
                                len3 = turn2;

                                start = from.getLeftState();
                                end = State(to.point, dir22);

                                type = DType::DIP_LRp;
                            }
                        }
                    }
                }
            }
        }


        {
            // vector from the center of the right turn to the goal point
            Vector diff = to.point - c1right;
            double diffAngle = diff.getAngle();
            double len = diff.length();

            // RS
            if (radius <= len) {
                double alpha = std::asin(radius / len);
                double directionS = diffAngle - alpha;
                double turn = angleToRight(from.getRight(), directionS);

                State st = from.getRightState();
                double lenS = (c1right + (from.point - c1right).rotate(turn)).distance(to.point);

                double nLen = lenS - radius * turn;
                if (nLen < length) {
                    if (to.inInterval(directionS)) {
                        length = nLen;
                        isCCC = false;
                        len1 = turn;
                        len2 = lenS;
                        len3 = 0;

                        start = st;
                        end = State(to.point, directionS);

                        type = DType::DIP_RS;
                    }
                }
            }

            // RL+
            if (radius < len && len < 3 * radius) {

                // angle in triangle 2R, R, len
                double alpha = std::acos((3 * radius * radius + len * len) / (4 * radius * len));

                { // first option + alpha
                    // direction between segments
                    double centerDir = diffAngle - M_PI / 2 - alpha;

                    // length of the first turning maneuver (L)
                    double turn1 = angleToRight(from.getRight(), centerDir);
                    Point centerPoint = (c1right + (from.point - c1right).rotate(turn1));

                    // R+
                    double dir2 = centerPoint.distance(to.point);
                    if (dir2 < 2 * radius) {
                        double alpha2 = std::asin(dir2 / 2 / radius);

                        double turn2 = 2 * M_PI - 2 * alpha2;
                        double dir22 = from.getRight() + turn1 + turn2;

                        double nLen = radius * (-turn1 + turn2);
                        if (nLen < length) {
                            if (to.inInterval(dir22)) {
                                length = nLen;
                                isCCC = false;
                                len1 = turn1;
                                len2 = 0;
                                len3 = turn2;

                                start = from.getRightState();
                                end = State(to.point, dir22);

                                type = DType::DIP_RLp;
                            }
                        }
                    }
                }
            }
        }

        ///////////////////////////////
        // CASE 3 - limited by second interval
        // SR, SL, L+R, R+L
        {
            // vector from the center of the right turn to the goal point
            Vector diff = c2right - from.point;
            double diffAngle = diff.getAngle();
            double len = diff.length();

            // SR
            if (radius <= len) {
                double alpha = std::asin(radius / len);
                double directionS = diffAngle + alpha;
                double turn = angleToRight(directionS, to.getLeft());

                double lenS = (c2right + (to.point - c2right).rotate(-turn)).distance(from.point);

                double nLen = lenS - radius * turn;

                if (nLen < length) {
                    if (from.inInterval(directionS)) {

                        length = nLen;
                        isCCC = false;
                        len1 = 0;
                        len2 = lenS;
                        len3 = turn;

                        start = State(from.point, directionS);
                        end = to.getLeftState();

                        type = DType::DIP_SR;
                    }
                }
            }

            // L+R
            if (radius < len && len < 3 * radius) {

                // angle in triangle 2R, R, len
                double alpha = std::acos((3 * radius * radius + len * len) / (4 * radius * len));

                { // first option + alpha
                    // direction between segments
                    double centerDir = diffAngle + M_PI / 2 + alpha;

                    // length of the second turning maneuver (R)
                    double turn2 = angleToRight(centerDir, to.getLeft());

                    // todo optimize
                    State fp = State(to.point, to.getLeft() + M_PI);
                    Arc back(fp, -turn2, radius);
                    Point centerPoint = back.getEnd().point;

                    // L+
                    double dir2 = centerPoint.distance(from.point);
                    if (dir2 < 2 * radius) {
                        double alpha2 = std::asin(dir2 / 2 / radius);

                        double turn1 = 2 * M_PI - 2 * alpha2;
                        double dir1 = to.getLeft() - turn1 - turn2;

                        double nLen = radius * (turn1 - turn2);
                        if (nLen < length) {
                            if (from.inInterval(dir1)) {
                                length = nLen;
                                isCCC = false;
                                len1 = turn1;
                                len2 = 0;
                                len3 = turn2;

                                start = State(from.point, dir1);
                                end = to.getLeftState();

                                type = DType::DIP_LpR;
                            }
                        }
                    }
                }
            }
        }


        {
            // vector from the center of the left turn to the goal point
            Vector diff = c2left - from.point;
            double diffAngle = diff.getAngle();
            double len = diff.length();

            // SL
            if (radius <= len) {
                double alpha = std::asin(radius / len);
                double directionS = diffAngle - alpha;
                double turn = angleToLeft(directionS, to.getRight());

                double lenS = (c2left + (to.point - c2left).rotate(-turn)).distance(from.point);

                double nLen = lenS + radius * turn;

                if (nLen < length) {
                    if (from.inInterval(directionS)) {
                        length = nLen;
                        isCCC = false;
                        len1 = 0;
                        len2 = lenS;
                        len3 = turn;

                        start = State(from.point, directionS);
                        end = to.getRightState();

                        type = DType::DIP_SL;
                    }
                }
            }

            // R+L
            if (radius < len && len < 3 * radius) {

                // angle in triangle 2R, R, len
                double alpha = std::acos((3 * radius * radius + len * len) / (4 * radius * len));

                { // first option + alpha
                    // direction between segments
                    double centerDir = diffAngle - M_PI / 2 - alpha;

                    // length of the second turning maneuver (L)
                    double turn2 = angleToLeft(centerDir, to.getRight());

                    // todo optimize
                    State fp = State(to.point, to.getRight() + M_PI);
                    Arc back(fp, -turn2, radius);
                    Point centerPoint = back.getEnd().point;

                    // L+
                    double dir2 = centerPoint.distance(from.point);
                    if (dir2 < 2 * radius) {
                        double alpha2 = std::asin(dir2 / 2 / radius);

                        double turn1 = -2 * M_PI + 2 * alpha2;
                        double dir1 = to.getRight() - turn1 - turn2;

                        double nLen = radius * (-turn1 + turn2);
                        if (nLen < length) {
                            if (from.inInterval(dir1)) {
                                length = nLen;
                                isCCC = false;
                                len1 = turn1;
                                len2 = 0;
                                len3 = turn2;

                                start = State(from.point, dir1);
                                end = to.getRightState();

                                type = DType::DIP_RpL;
                            }
                        }
                    }
                }
            }
        }
    }

    void Dubins::initForDIP(const State &from, const State &to, const Vector & dir1, const Vector & dir2) {
        bool used = false;

        //Vector dir1 = from.getNormalizedDirection();
        //Vector dir2 = to.getNormalizedDirection();

        Vector dir1radius = dir1 * radius;
        Vector dir2radius = dir2 * radius;

        Point c1left = from.point + dir1radius.left();
        Point c1right = from.point + dir1radius.right();

        Point c2left = to.point + dir2radius.left();
        Point c2right = to.point + dir2radius.right();

        double n1, n2, n3, nLength, centerDistance;

        // RSR - maneuver

        Vector diff = c2right - c1right;
        double ang = diff.getAngle();

        n1 = angleToRight(from.ang, ang);
        n2 = diff.length();
        n3 = angleToRight(ang, to.ang);

        n1 = checkToleranceRight(n1);
        n3 = checkToleranceRight(n3);

        nLength = n2 + radius * (std::fabs(n1) + std::fabs(n3));

        if (nLength < length) {
            len1 = n1;
            len2 = n2;
            len3 = n3;
            length = nLength;
            used = true;
            isCCC = false;
            type = DType::DIP_RSR;
        }

        // LSL - maneuver

        diff = c2left - c1left;
        ang = diff.getAngle();

        n1 = angleToLeft(from.ang, ang);
        n2 = diff.length();
        n3 = angleToLeft(ang, to.ang);

        n1 = checkToleranceLeft(n1);
        n3 = checkToleranceLeft(n3);

        nLength = n2 + radius * (std::fabs(n1) + std::fabs(n3));

        if (nLength < length) {
            len1 = n1;
            len2 = n2;
            len3 = n3;
            length = nLength;
            used = true;
            isCCC = false;
            type = DType::DIP_LSL;
        }

        // LSR - maneuver

        diff = c2right - c1left;
        centerDistance = diff.length();

        if (centerDistance * (1.0) >= 2 * radius) {
            double alpha = std::asin(std::fmin(1, 2 * radius / centerDistance));
            double centerAngle = std::atan2(c2right.y - c1left.y, c2right.x - c1left.x) + alpha;
            n2 = std::sqrt(std::fmax(0, centerDistance * centerDistance - 4 * radius * radius));

            // normalize angle
            n1 = angleToLeft(from.ang, centerAngle);
            n3 = angleToRight(centerAngle, to.ang);

            n1 = checkToleranceLeft(n1);
            n3 = checkToleranceRight(n3);

            nLength = n2 + radius * (std::fabs(n1) + std::fabs(n3));

            if (nLength < length) {
                len1 = n1;
                len2 = n2;
                len3 = n3;
                length = nLength;
                used = true;
                isCCC = false;
                type = DType::DIP_LSR;
            }
        }

        // RSL - maneuver

        diff = c2left - c1right;
        centerDistance = diff.length();

        if (centerDistance * (1.0) >= 2 * radius) {
            double alpha = std::asin(std::fmin(1, 2 * radius / centerDistance));
            double centerAngle = std::atan2(c2left.y - c1right.y, c2left.x - c1right.x) - alpha;
            n2 = std::sqrt(std::fmax(0, centerDistance * centerDistance - 4 * radius * radius));

            // normalize angle
            n1 = angleToRight(from.ang, centerAngle);
            n3 = angleToLeft(centerAngle, to.ang);

            n1 = checkToleranceRight(n1);
            n3 = checkToleranceLeft(n3);

            nLength = n2 + radius * (std::fabs(n1) + std::fabs(n3));

            if (nLength < length) {
                len1 = n1;
                len2 = n2;
                len3 = n3;
                length = nLength;
                used = true;
                isCCC = false;
                type = DType::DIP_RSL;
            }
        }

        // CCC maneuver is possible only in case start and end state is close enougth
        if ((from.point - to.point).length() <= 4 * radius) {

            // RLR - maneuver
            diff = c2right - c1right;
            double diffAngle = diff.getAngle();
            centerDistance = diff.length();

            if (centerDistance <= 4 * radius) {
                // direction of Vector(S1,S2) to Vector(S1,S3)
                double alpha = std::acos(centerDistance / radius / 4);

                // direction between first and second arc
                double dir12 = diffAngle - M_PI / 2 - alpha;
                // direction between second and third arc
                double dir23 = diffAngle + M_PI / 2 + alpha;

                n1 = angleToRight(from.ang, dir12);
                n2 = angleToLeft(dir12, dir23);
                n3 = angleToRight(dir23, to.ang);

                nLength = radius * (std::fabs(n1) + std::fabs(n2) + std::fabs(n3));

                if (nLength < length) {
                    isCCC = true;
                    len1 = n1;
                    len2 = n2;
                    len3 = n3;
                    length = nLength;
                    used = true;
                    type = DType::DIP_RLR;
                }
            }

            // LRL - maneuver
            diff = c2left - c1left;
            diffAngle = diff.getAngle();
            centerDistance = diff.length();

            if (centerDistance <= 4 * radius) {
                // direction of Vector(S1,S2) to Vector(S1,S3)
                double alpha = std::acos(centerDistance / radius / 4);

                // direction between first and second arc
                double dir12 = diffAngle + M_PI / 2 + alpha;
                // direction between second and third arc
                double dir23 = diffAngle - M_PI / 2 - alpha;

                n1 = angleToLeft(from.ang, dir12);
                n2 = angleToRight(dir12, dir23);
                n3 = angleToLeft(dir23, to.ang);

                nLength = radius * (std::fabs(n1) + std::fabs(n2) + std::fabs(n3));

                if (nLength < length) {
                    isCCC = true;
                    len1 = n1;
                    len2 = n2;
                    len3 = n3;
                    length = nLength;
                    used = true;
                    type = DType::DIP_LRL;
                }
            }
        }

        if (used) {
            start = from;
            end = to;
        }
    }

}
