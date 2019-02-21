/**
 * @file dip.cpp
 * @author Petr Vana
 * @brief Implementation of the Generalized Dubins Interval Problem (GDIP)
 * - shortest path connecting two circle regions with heading intervals
 */

#include <iomanip>
#include "dubins.h"

using namespace std;

namespace opendubins {

    Dubins::Dubins(AngleInterval from, AngleInterval to, double newRadius, double diff1, double diff2) {
        radius = newRadius;
        length = std::numeric_limits<double>::max();
        type = DType::Unknown;
        initGDIP(from, to, diff1, diff2);
    }

    Dubins::Dubins(AngleInterval from, AngleInterval to, double newRadius, double diff) {
        radius = newRadius;
        length = std::numeric_limits<double>::max();
        type = DType::Unknown;
        initGDIP(from, to, diff);
    }

    void Dubins::GDIP_S(const AngleInterval &from, const AngleInterval &to, const double &diff2) {
        // ends of intervals:
        // from(A,B)
        const auto A = from.getRight();
        const auto B = from.getLeft();
        //  to(C,D)
        const auto C = to.getRight();
        const auto D = to.getLeft();

        Vector diff = to.point - from.point;
        double diffAng = diff.getAngle();

        if (diff.length() < diff2) { // no move needed
            double XX[] = {A, B, C, D};
            for(auto X : XX) {
                if (from.inInterval(X) && to.inInterval(X)) {
                    isCCC = false;
                    length = len1 = len2 = len3 = 0;
                    double ang = X;
                    start = State(from.point, ang);
                    end = State(from.point, ang);
                    type = DType::GDIP_NO;
                    //USED(GDIP, 1);
                }
            }
        } else {
            if (from.inInterval(diffAng) && to.inInterval(diffAng)) { // the shortest S maneuver is in interval
                auto intersection = to.point + diff.normalize() * -diff2;
                auto nLen = intersection.distance(from.point);
                if (nLen < length) {
                    isCCC = false;
                    len1 = len3 = 0;
                    length = len2 = nLen;
                    start = State(from.point, diffAng);
                    end = State(intersection, diffAng);
                    type = DType::GDIP_S;
                    //USED(GDIP, 2);
                }
            } else { // solution is at the edge of the interval
                double XX[] = {A, B, C, D};
                for(auto X : XX) {
                    if (from.inInterval(X) && to.inInterval(X)) {
                        GDIP_S_onEdge(from, to.point, diff2, X);
                    }
                }
            } // else there is no intersection with the neighborhood circle
        }
    }

    // todo - remove because it is not necessary - REALY ???
    void Dubins::GDIP_S_onEdge(const AngleInterval &from, const Point &to, const double &diff2, double dir) {
        Circle ng = Circle(to, diff2);

        State halfLine1 = State(from.point, dir);
        auto intersection = ng.halfLineIntersection(halfLine1);
        if (intersection.isValid()) {
            auto nLen = from.point.distance(intersection);
            if (nLen < length) {
                isCCC = false;
                len1 = len3 = 0;
                length = len2 = nLen;
                start = State(from.point, halfLine1.ang);
                end = State(intersection, halfLine1.ang);
                type = DType::GDIP_S;
                //USED(GDIP, 3);
            }
        }
    }

    void Dubins::GDIP_CS(const State &from, const AngleInterval &to, const double &diff2, bool isLeft) {

        const auto totalDiff = diff2;

        Vector dir1radius = from.getNormalizedDirection() * radius;
        // center of the turning circle
        Point center = from.point + (isLeft ? dir1radius.left() : dir1radius.right());

        // vector from the center of the right turn to the goal point
        Vector diff = to.point - center;

        double len = diff.length();
        if (radius <= len) { // RS maneuver
            double alpha = asin(radius / len);
            double directionS = diff.getAngle() + (isLeft ? alpha : -alpha);
            double turn = isLeft ? angleToLeft(from.ang, directionS) : angleToRight(from.ang, directionS);

            double lenS = (center + (from.point - center).rotate(turn)).distance(to.point);

            // todo - to be tested
            if (to.inInterval(directionS)) {
                // end of R segments leads directly to center of the goal area (to.point)
                auto lenS2 = max(0.0, lenS - totalDiff);
                //auto lenS2 = lenS;
                double nLen = lenS2 + radius * fabs(turn);
                if (nLen < length) {
                    length = nLen;
                    isCCC = false;
                    len1 = turn;
                    len2 = lenS2;
                    len3 = 0;
                    start = from;
                    end = State(to.point - Vector(directionS) * (lenS - lenS2), directionS);
                    type = isLeft ? DType::GDIP_LS : DType::GDIP_RS;
                    //USED(GDIP, 4);
                }
            }
        }

        {
            // termination angle is on right edge of the termination interval
            GDIP_CS_onEdge(from, to, diff2, to.getLeft(), isLeft);

            // termination angle is on left edge of the termination interval
            GDIP_CS_onEdge(from, to, diff2, to.getRight(), isLeft);
        }

        { // possible C maneuver
            Circle goal(to.point, diff2);

            { // C maneuver ends when intersecting goal area (circle)
                Arc arc(from, (isLeft ? 2 : -2) * M_PI, radius);

                State interState;
                double interDist;
                tie(interState, interDist) = goal.firstIntersection(arc);

                if (interState.point.isValid()) {
                    auto nLen = interDist;
                    if (to.inInterval(interState.ang)) {
                        if (nLen < length) {
                            length = nLen;
                            isCCC = false;
                            len1 = (isLeft ? 1 : -1) * nLen / radius;
                            len2 = 0;
                            len3 = 0;
                            start = from;
                            end = interState;
                            type = isLeft ? DType::GDIP_L : DType::GDIP_R;
                            //USED(GDIP, 5);
                        }
                    }
                }
            }

            { // C maneuver ends when the terminal heading interval is met
                double terminalAngle = isLeft ? to.getRight() : to.getLeft();
                double turn = isLeft ? angleToLeft(from.ang, terminalAngle) : angleToRight(from.ang, terminalAngle);

                double nLen = radius * fabs(turn);
                if (nLen < length) {
                    Arc arc(from, turn, radius);
                    auto newEnd = arc.getEnd();

                    if (goal.pointInCircle(newEnd.point)) {
                        length = nLen;
                        isCCC = false;
                        len1 = turn;
                        len2 = 0;
                        len3 = 0;
                        start = from;
                        end = newEnd;
                        type = isLeft ? DType::GDIP_L : DType::GDIP_R;
                        //USED(GDIP, 6);
                    }

                }
            }
        }
    }

    void Dubins::GDIP_CS_onEdge(const State &from, const AngleInterval &to, const double &diff2, double dir, const bool& isLeft) {
        Circle goal(to.point, diff2);

        auto turn2 = isLeft ? angleToLeft(from.ang, dir) : angleToRight(from.ang, dir);

        Point S1 = from.point + Vector(from.ang + M_PI/2) * (isLeft ? radius : -radius);
        State rEnd2(S1 + (from.point - S1).rotate(turn2), from.ang + turn2);

        // todo remove
       // auto rEnd2 = Arc(from, turn2, RADIUS).getEnd();

        auto intersect2 = goal.halfLineIntersection(rEnd2);

        if (intersect2.isValid()) {
            auto lenS = intersect2.distance(rEnd2.point);
            auto nLen = lenS + radius * fabs(turn2);
            if (nLen < length) {
                length = nLen;
                isCCC = false;
                len1 = turn2;
                len2 = lenS;
                len3 = 0;
                start = from;
                end = State(intersect2, rEnd2.ang);
                type = isLeft ? DType::GDIP_LS : DType::GDIP_RS;
                //USED(GDIP, 4);
            }
        }
    }

    void Dubins::GDIP_SC(const AngleInterval& from, const AngleInterval& to, const double& diff2) {
        Dubins act;
        act.radius = radius;

        AngleInterval revertedFrom = from.reverse();
        AngleInterval revertedTo = to.reverse();

        // reverted S and SL maneuver ... to > from
        act.GDIP_CS(revertedTo.getLeftState(), revertedFrom, diff2, true);

        // reverted S and SR maneuver ... to > from
        act.GDIP_CS(revertedTo.getRightState(), revertedFrom, diff2, false);

        // todo - different side as a special case of Cp
        // toto - test this
        // reverted S and SL maneuver ... to > from
        //act.GDIP_CS(revertedTo.getRightState(), revertedFrom, diff2, true);
        // reverted S and SR maneuver ... to > from
        //act.GDIP_CS(revertedTo.getLeftState(), revertedFrom, diff2, false);
        // todo

        if(act.length < length){
            length = act.length;
            isCCC = act.isCCC;
            len1 = -act.len3;
            len2 = act.len2;
            len3 = -act.len1;
            start = act.end.reverse();
            start.point = from.point;
            end = act.start.reverse();
            end.point = act.start.point - (act.end.point - from.point);
            type = DType::Unknown;
            if(act.type == DType::GDIP_L){
                type = DType::GDIP_R;
            }else if(act.type == DType::GDIP_R) {
                type = DType::GDIP_L;
            }else if(act.type == DType::GDIP_RS){
                type = DType::GDIP_SL;
            }else if(act.type == DType::GDIP_LS){
                type = DType::GDIP_SR;
            }
        }
    }

    void Dubins::GDIP_CSC_same_side(const State &from, const State &to, const double &diff2, const bool &isLeft) {
        double totalTurn = angleToSide(from.ang, to.ang, isLeft);

        Point target = Arc(from, totalTurn, radius).getEnd().point;
        double rightDir;
        double diff;
        if(isLeft){
            rightDir = from.ang;
            diff = totalTurn;
        }else{
            rightDir = to.ang;
            diff = -totalTurn;
        }

        AngleInterval targetInterval = AngleInterval(target, rightDir, diff);
        Circle goal(to.point, diff2);

        // find shortest point in the goal area from target
        // direction from terget needs to be in targetInterval
        {
            Vector dir = goal.shortestVectorInSector(targetInterval);
            double dirAng = dir.getAngle();

            double turn1 = angleToSide(from.ang, dirAng, isLeft);
            double straight = dir.length();
            double turn2 = angleToSide(dirAng, to.ang, isLeft);

            double nLen = radius * (fabs(turn1) + fabs(turn2)) + straight;

            if (nLen < length) {
                length = nLen;
                isCCC = false;
                len1 = turn1;
                len2 = straight;
                len3 = turn2;
                type = isLeft ? DType::GDIP_LSL : DType::GDIP_RSR;
                start = from;
                end = getSecondArc().getEnd();
            }
        }

        // full turn
        {
            Vector dir = goal.center - targetInterval.point;
            double dirAng = dir.getAngle();

            double turn1 = angleToSide(from.ang, dirAng, isLeft);
            double straight = max(0.0, dir.length() - goal.radius);
            double turn2 = angleToSide(dirAng, to.ang, isLeft);

            double nLen = radius * (fabs(turn1) + fabs(turn2)) + straight;

            if (nLen < length) {
                length = nLen;
                isCCC = false;
                len1 = turn1;
                len2 = straight;
                len3 = turn2;
                type = isLeft ? DType::GDIP_LSL : DType::GDIP_RSR;
                start = from;
                end = getSecondArc().getEnd();
            }
        }
    }

    void Dubins::Dubins_StateToCircle(const State& from, const Circle& goal, const double& radius, const bool& isLeft,
                              double& turn, double& straight, double& length) {

        double sgn = isLeft ? 1 : -1;

        turn = NAN;
        straight = NAN;
        length = numeric_limits<double>::max();

        // no maneuver
        if(goal.pointInCircle(from.point)){
            turn = 0;
            straight = 0;
            length = 0;
            return;
        }

        Arc arc(from, sgn, radius);

        // C case - intersection of the goal and the arc

        State interState;
        double interDist;
        tie(interState, interDist) = goal.firstIntersection(arc);

        if(!std::isnan(interState.point.x)){
            turn = interDist / radius * sgn;
            straight = 0;
            length = interDist;
        }

        // CS case - to the goal center
        Vector dir1radius = from.getNormalizedDirection() * radius;
        // center of the turning circle
        Point c1right = from.point + dir1radius.right();
        Point c1left = from.point + dir1radius.left();

        // vector from the center of the right turn to the goal point
        Vector diff = goal.center - (isLeft ? c1left : c1right);

        double len = diff.length();
        if (radius <= len) { // RS maneuver
            double alpha = asin(radius / len);
            double directionS = diff.getAngle() + (isLeft ? alpha : -alpha);
            double nTurn = angleToSide(from.ang, directionS, isLeft);

            State st = from;
            Arc arc(st, nTurn, radius);
            double lenS = arc.getEnd().point.distance(goal.center);

            // end of R segments leads directly to center of the goal area (to.point)
            auto lenS2 = max(0.0, lenS - goal.radius);
            double nLen = lenS2 + radius * fabs(nTurn);
            if (nLen < length) {
                length = nLen;
                turn = nTurn;
                straight = lenS2;
            }
        }

    }

    void Dubins::GDIP_CSC_diff_side(const State &from, const State &to, const double &diff2, const bool &isLeft) {
        // the first turn is longer
        {
            double turn1 = angleToSide(from.ang, to.ang, isLeft);
            State from2 = Arc(from, turn1, radius).getEnd();

            Point halfGoalPoint = from2.point + ((to.point - from2.point) * 0.5);
            Circle halfGoal(halfGoalPoint, diff2 / 2);

            double turnSTC, straightSTC, lengthSTC;
            Dubins_StateToCircle(from2, halfGoal, radius, isLeft, turnSTC, straightSTC, lengthSTC);

            double nLen = radius*(fabs(turn1) + 2*fabs(turnSTC)) + 2*straightSTC;
            if(nLen < length){
                length = nLen;
                len1 = turn1 + turnSTC;
                len2 = straightSTC * 2;
                len3 = -turnSTC;
                isCCC = false;
                start = from;
                end = getSecondArc().getEnd();
                type = isLeft ? DType::GDIP_LSR : DType::GDIP_RSL;
            }
        }

        // the second turn is longer
        {
            double turn2 = angleToSide(from.ang, to.ang, !isLeft);
            State to2 = Arc(to.reverse(), -turn2, radius).getEnd().reverse();

            Point halfGoalPoint = from.point + ((to2.point - from.point) * 0.5);
            Circle halfGoal(halfGoalPoint, diff2 / 2);

            double turnSTC, straightSTC, lengthSTC;
            Dubins_StateToCircle(from, halfGoal, radius, isLeft, turnSTC, straightSTC, lengthSTC);

            double nLen = radius*(fabs(turn2) + 2*fabs(turnSTC)) + 2*straightSTC;
            if(nLen < length){
                length = nLen;
                len1 = turnSTC;
                len2 = straightSTC * 2;
                len3 = -turnSTC + turn2;
                isCCC = false;
                start = from;
                end = getSecondArc().getEnd();
                type = isLeft ? DType::GDIP_LSR : DType::GDIP_RSL;
            }
        }

    }

    void Dubins::GDIP_Lp_Rp(const AngleInterval &from, const AngleInterval & to_center, const double &diff2) {

        AngleInterval to = to_center;
        to.point += (to.point - from.point).normalize() * diff2;

        const Vector dir = to.point - from.point;
        const double dir_length = dir.length();
        const double dir_ang = dir.getAngle();

        // L+, R+
        if (dir_length < 2 * radius) {
            double alpha = asin(dir_length / 2 / radius);

            // L+
            {
                double dir1 = dir_ang + M_PI + alpha;
                double dir2 = dir_ang + M_PI - alpha;

                double turn = angleToLeft(dir1, dir2);

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

                        type = DType::GDIP_Lp;
                    }
                }
            }

            // R+
            {
                double dir1 = dir_ang + M_PI - alpha;
                double dir2 = dir_ang + M_PI + alpha;

                double turn = angleToRight(dir1, dir2);

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

                        type = DType::GDIP_Rp;
                    }
                }
            }
        }

    }

    double GDIP_CCp_fce(const State &from, const AngleInterval &to, const double &diff2, bool isLeft, double x, const Point& S1, double radius){
        Point endPoint = to.point + Vector(x) * diff2;

        // vector from first center to target point
        Vector diff = endPoint - S1;

        double len = diff.length();
        if (radius < len && len < 3 * radius) {

            // angle in triangle 2R, R, len
            double alpha = acos((3 * radius * radius + len * len) / (4 * radius * len));

            // first option + alpha
            // direction between segments
            double centerDir = (isLeft ? diff.left() : diff.right()).getAngle() + (isLeft ? alpha : -alpha);

            // length of the first turning maneuver (L)
            double turn1;
            if (isLeft) {
                turn1 = angleToLeft(from.ang, centerDir);
            } else {
                turn1 = angleToRight(from.ang, centerDir);
            }

            Point centerPoint = S1 + (from.point - S1).rotate(turn1);

            double bigTurnDst = centerPoint.distance(endPoint);
            if (bigTurnDst < 2 * radius) {
                double alpha2 = asin(bigTurnDst / 2 / radius);
                double turn2 = -2 * M_PI + 2 * alpha2;
                double nLen = radius * (std::fabs(turn1) + std::fabs(turn2));
                return nLen;
            }

        }
        return NAN;
    }

#define GDIP_CCp_MACRO(x) GDIP_CCp_fce(from, to, diff2, isLeft, x, S1, radius);

    void Dubins::GDIP_CCp(const State &from, const AngleInterval &to, const double &diff2, bool isLeft) {
        //first segment center
        Point S1 = from.point + (isLeft ? radius : -radius) * from.getNormalizedDirection().left();

        auto dir = to.point - S1;

        double vbest = NAN;
        double best = NAN;

        double a = dir.getAngle();
        double b = a + M_PI / 2 * (isLeft ? -1: 1);
        double c = (a+b)/2;

        double variants[] = {a, b, c};
        for(double i : variants){
            double vi = GDIP_CCp_MACRO(i);
            if(std::isnan(best) || (!std::isnan(vi) && vi < vbest)) {
                best = i;
                vbest = vi;
            }
        }

        if(std::isnan(best)) return;

        double step = 0.01;

        // local search
        while (std::fabs(step) > TOLERANCE / 10) {
            double act = best + step;
            double vact = GDIP_CCp_MACRO(act);

            if (vact < vbest) {
                vbest = vact;
                best = act;
                // todo - optimize coefficients
                step *= 6;
            } else {
                step /= -10;
            }
        }

        Point endPoint = to.point + Vector(best) * diff2;

        // vector from first center to target point
        Vector diff = endPoint - S1;

        double len = diff.length();
        if (radius < len && len < 3 * radius) {

            // angle in triangle 2R, R, len
            double alpha = acos((3 * radius * radius + len * len) / (4 * radius * len));

            // first option + alpha
            // direction between segments
            double centerDir = (isLeft ? diff.left() : diff.right()).getAngle() + (isLeft ? alpha : -alpha);

            // length of the first turning maneuver (L)
            double turn1;
            if (isLeft) {
                turn1 = angleToLeft(from.ang, centerDir);
            } else {
                turn1 = angleToRight(from.ang, centerDir);
            }

            Point centerPoint = S1 + (from.point - S1).rotate(turn1);

            double bigTurnDst = centerPoint.distance(endPoint);
            if (bigTurnDst < 2 * radius) {
                double alpha2 = asin(bigTurnDst / 2 / radius);

                double turn2 = -2 * M_PI + 2 * alpha2;
                turn2 *= (isLeft ? 1 : -1);
                double dir2 = from.ang + turn1 + turn2;

                double nLen = radius * (std::fabs(turn1) + std::fabs(turn2));
                if (nLen < length) {
                    if (to.inInterval(dir2)) {
                        length = nLen;
                        isCCC = false;
                        len1 = turn1;
                        len2 = 0;
                        len3 = turn2;

                        start = from;
                        end = State(endPoint, dir2);

                        type = isLeft ? DType::GDIP_LRp : DType::GDIP_RLp;
                    }
                }
            }
        }
    }

    double GDIP_CpC_fce(const AngleInterval &from, const State &to, const double &diff2, bool isLeft, double x, const Point& S2, double radius){
        Point startPoint = from.point + Vector(x) * diff2;

        // vector from the second center to the start point
        Vector diff = startPoint - S2;

        double len = diff.length();
        if (radius < len && len < 3 * radius) {

            // angle in triangle 2R, R, len
            double alpha = acos((3 * radius * radius + len * len) / (4 * radius * len));

            // first option + alpha
            // direction between segments
            double centerDir = (isLeft ? diff.right() : diff.left()).getAngle() + (isLeft ? alpha : -alpha);

            // length of the second turning maneuver C
            double turn2;
            if (isLeft) {
                turn2 = angleToRight(centerDir, to.ang);
            } else {
                turn2 = angleToLeft(centerDir, to.ang);
            }

            Point centerPoint = S2 + (to.point - S2).rotate(-turn2);

            double bigTurnDst = centerPoint.distance(startPoint);
            if (bigTurnDst < 2 * radius) {
                double alpha2 = asin(bigTurnDst / 2 / radius);
                double turn1 = 2 * M_PI - 2 * alpha2;
                double nLen = radius * (std::fabs(turn1) + std::fabs(turn2));
                return nLen;
            }

        }
        return NAN;
    }

#define GDIP_CpC_MACRO(x) GDIP_CpC_fce(from, to, diff2, isLeft, x, S2, radius);

    void Dubins::GDIP_CpC(const AngleInterval &from, const State &to, const double &diff2, bool isLeft) {
        // second (shorter) segment center
        Point S2 = to.point + (isLeft ? radius : -radius) * to.getNormalizedDirection().right();

        auto dir = from.point - S2;

        double vbest = NAN;
        double best = NAN;

        double a = dir.getAngle();
        double b = a + M_PI / 2 * (isLeft ? 1: -1);
        double c = (a+b)/2;

        double variants[] = {a, b, c};
        for(double i : variants){
            double vi = GDIP_CpC_MACRO(i);
            if(std::isnan(best) || (!std::isnan(vi) && vi < vbest)) {
                best = i;
                vbest = vi;
            }
        }

        if(std::isnan(best)) return;

        double step = 0.01;

        // local search
        while (std::fabs(step) > TOLERANCE / 10) {
            double act = best + step;
            double vact = GDIP_CpC_MACRO(act);

            if (vact < vbest) {
                vbest = vact;
                best = act;
                // todo - optimize coefficients
                step *= 6;
            } else {
                step /= -10;
            }
        }

        Point startPoint = from.point + Vector(best) * diff2;

        // vector from the second center to the start point
        Vector diff = startPoint - S2;

        double len = diff.length();
        if (radius < len && len < 3 * radius) {

            // angle in triangle 2R, R, len
            double alpha = acos((3 * radius * radius + len * len) / (4 * radius * len));

            // first option + alpha
            // direction between segments
            double centerDir = (isLeft ? diff.right() : diff.left()).getAngle() + (isLeft ? alpha : -alpha);

            // length of the second turning maneuver C
            double turn2;
            if (isLeft) {
                turn2 = angleToRight(centerDir, to.ang);
            } else {
                turn2 = angleToLeft(centerDir, to.ang);
            }

            Point centerPoint = S2 + (to.point - S2).rotate(-turn2);

            double bigTurnDst = centerPoint.distance(startPoint);
            if (bigTurnDst < 2 * radius) {
                double alpha2 = asin(bigTurnDst / 2 / radius);

                double turn1 = 2 * M_PI - 2 * alpha2;
                turn1 *= (isLeft ? 1 : -1);
                double dir1 = to.ang - turn1 - turn2; // reverse calculation

                double nLen = radius * (std::fabs(turn1) + std::fabs(turn2));
                if (nLen < length) {
                    if (from.inInterval(dir1)) {
                        length = nLen;
                        isCCC = false;
                        len1 = turn1;
                        len2 = 0;
                        len3 = turn2;

                        start = State(from.point, dir1);
                        end = to;
                        end.point -= (startPoint - from.point);

                        type = isLeft ? DType::GDIP_LpR : DType::GDIP_RpL;
                    }
                }
            }

        }
    }

    void Dubins::GDIP_CCC(const State &from, const State &to, const double &diff2, const bool &isLeft) {
        // first center
        Point S1 = from.point + from.getNormalizedDirection().left() * (isLeft ? radius : -radius);

        // todo - move this point
        // third center
        Point S3 = to.point + to.getNormalizedDirection().left() * (isLeft ? radius : -radius);
        S3 = S3 + (S3 - S1).normalize() * diff2;

        // now, we need to find the second center S2

        auto diff = S3 - S1;
        double centerDistance = diff.length();

        if (centerDistance < 4 * radius) {
            // direction of Vector(S1,S2) to Vector(S1,S3)
            double alpha = acos(centerDistance / radius / 4);

            // tmp variable
            double ma = M_PI / 2 + alpha;
            // direction between first and second arc
            double dir12 = diff.getAngle() + (isLeft ? ma : -ma);
            // direction between second and third arc
            double dir23 = diff.getAngle() - (isLeft ? ma : -ma);

            // length of all three turns
            double n1 = (isLeft ? angleToLeft(from.ang, dir12) : angleToRight(from.ang, dir12));
            double n2 = (isLeft ? angleToRight(dir12, dir23) : angleToLeft(dir12, dir23));
            double n3 = (isLeft ? angleToLeft(dir23, to.ang) : angleToRight(dir23, to.ang));

            double nLength = radius * (std::fabs(n1) + std::fabs(n2) + std::fabs(n3));

            if (nLength < length) {
                isCCC = true;
                len1 = n1;
                len2 = n2;
                len3 = n3;
                length = nLength;
                start = from;
                Point endPoint = S3 + (isLeft ? -radius : radius) * Vector(to.ang).left();
                end = State(endPoint, to.ang);
                type = (isLeft ? DType::GDIP_LRL : DType::GDIP_RLR);
            }
        }
    }

    void Dubins::initGDIP(AngleInterval from, AngleInterval to, double diff2) {

        double dst = from.point.distance(to.point);

        // TODO - enable to improve numeric stability, should not be necessary
        //initIntervalProblem(from, to);

        // S maneuver
        GDIP_S(from, to, diff2);

        // L and LS maneuver
        GDIP_CS(from.getLeftState(), to, diff2, true);
        // R and RS maneuver
        GDIP_CS(from.getRightState(), to, diff2, false);

        // SL nad SR maneuvers
        GDIP_SC(from, to, diff2);

        // LSL - same side
        GDIP_CSC_same_side(from.getLeftState(), to.getRightState(), diff2, true);
        // RSR - same side
        GDIP_CSC_same_side(from.getRightState(), to.getLeftState(), diff2, false);
        // LSR - same side
        GDIP_CSC_diff_side(from.getLeftState(), to.getLeftState(), diff2, true);
        // RSL - same side
        GDIP_CSC_diff_side(from.getRightState(), to.getRightState(), diff2, false);

        // Lp and Rp maneeuvers
        if(length > M_PI * radius && dst + diff2 < 2*radius) {
            GDIP_Lp_Rp(from, to, diff2);
        }

        // LpRp and RpLp maneuvers are not optimal - see paper
        if(length > M_PI * radius && dst - diff2 < 4*radius) {
            // LRL maneuver
            GDIP_CCC(from.getLeftState(), to.getRightState(), diff2, true);
            // RLR maneuver
            GDIP_CCC(from.getRightState(), to.getLeftState(), diff2, false);
        }

        if(length > M_PI * radius && dst - diff2 < 4*radius) {
            // LRp maneuver
            GDIP_CCp(from.getLeftState(), to, diff2, true);
            // RLp maneuver
            GDIP_CCp(from.getRightState(), to, diff2, false);
            // LpR maneuver
            GDIP_CpC(from, to.getLeftState(), diff2, true);
            // RpL maneuver
            GDIP_CpC(from, to.getRightState(), diff2, false);
        }

        // TODO - implement other types

        // todo - test this
        // special case of Cp maneuver - different side than CS
        // L and LS maneuver
        //GDIP_CS(from.getRightState(), to, diff2, true);
        // R and RS maneuver
        //GDIP_CS(from.getLeftState(), to, diff2, false);
        // todo - test this

    }

    void Dubins::initGDIP(AngleInterval from, AngleInterval to, double diff1, double diff2) {

        initGDIP(from, to, diff1 + diff2);

        // change positions of start and end to meet distance conditions (diff1 and diff2)
        auto diffVector1 = start.point - from.point;
        auto diffVector2 = end.point - to.point;

        auto diff = diff1 + diff2;
        if (diff > 0) {
            auto alpha1 = diff1 / diff;
            auto alpha2 = 1 - alpha1;

            start.point = from.point + alpha1 * diffVector1 - alpha1 * diffVector2;
            end.point = to.point + alpha2 * diffVector2 - alpha2 * diffVector1;
        }
    }

}
