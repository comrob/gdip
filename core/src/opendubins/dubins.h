/**
 * @file dubins.h
 * @author Petr Vana
 * @brief Header file for Dubins maneuver
 */

#pragma once

#include "dtype.h"
#include "line.h"
#include "arc.h"
#include "angleinterval.h"
#include "circle.h"

#define USE_XpXp 0

namespace opendubins {

    class Dubins : public Path {

    private:
        // in radians (positive - left, negative - right) or in meters (straight)
        double len1, len2, len3;

        DType type;

    public:
        State start;
        State end;

        double radius;

        bool isCCC;

        double length;

        // default constructor
        Dubins();

        // create Dubins from a straight line (RADIUS = 1)
        Dubins(const Line& line);

        // classical Dubins maneuver
        Dubins(const State& from, const State& to, const double& radius);

        // Dubins Interval Problem (DIP)
        Dubins(AngleInterval from, AngleInterval to, double radius);

        // Generalited Dubins Interval Problem (GDIP)
        // diff1 - neighborhood around the start state
        // diff2 - neighborhood around the end state
        Dubins(AngleInterval from, AngleInterval to, double radius, double diff1, double diff2);

        // Generalited Dubins Interval Problem (GDIP)
        // diff - neighborhood around the end state
        Dubins(AngleInterval from, AngleInterval to, double radius, double diff2);

        Dubins(State pos, bool isCCC, double le1, double le2, double le3, double radius);

        virtual ~Dubins();

        bool check();

        virtual State getState(double len) const;

        virtual StateAtDistance getClosestStateAndDistance(const Point &p) const;

        Arc getFirstArc() const;

        Line getCenter() const;

        Arc getCenterArc() const;

        Arc getSecondArc() const;

        StateAtDistance intersectLine(const Line &line) const;

        bool intersectLineBool(const Line &line) const;

        inline double getLength() const {
            return length;
        }

        inline bool getIsCCC() const {
            return isCCC;
        }

        inline double getLen1() const {
            return len1;
        }

        inline double getLen2() const {
            return len2;
        }

        inline double getLen3() const {
            return len3;
        }

        inline double getRadius() const {
            return radius;
        }

        inline void calculateLength() {
            if (isCCC) {
                length = radius * (std::fabs(len1) + std::fabs(len2) + std::fabs(len3));
            } else {
                length = len2 + radius * (std::fabs(len1) + std::fabs(len3));
            }
        }

        inline double getTotalTurning() {
            return std::fabs(len1) + std::fabs(len3);
        }

        inline DType getType() const {
            return type;
        }

    private:
        // create standard Dubins maneuver
        void init(const State & from, const State & to);
        // solve DIP
        void initIntervalProblem(const AngleInterval & from, const AngleInterval & to);
        // solve OS-GDIP
        void initGDIP(AngleInterval from, AngleInterval to, double diff);
        // convert GDIP to one-side GDIP (OS-GDIP)
        void initGDIP(AngleInterval from, AngleInterval to, double diff1, double diff2);

        // private functions for solving DIP ------------------

        // create standard Dubins maneuver for DIP using pre-computed values
        void initForDIP(const State & from, const State & to, const Vector & dir1, const Vector & dir2);

        // private functions for solving GDIP ------------------
        void GDIP_S(const AngleInterval& from, const AngleInterval& to, const double &diff2);
        void GDIP_S_onEdge(const AngleInterval& from, const Point& to, const double& diff2, double dir);

        void GDIP_CS(const State& from, const AngleInterval& to, const double &diff2, bool isLeft);
        void GDIP_CS_onEdge(const State& from, const AngleInterval& to, const double& diff2, double dir, const bool& isLeft);

        void GDIP_SC(const AngleInterval& from, const AngleInterval& to, const double& diff2);

        void GDIP_CSC_same_side(const State &from, const State &to, const double &diff2, const bool &isLeft);
        void GDIP_CSC_diff_side(const State &from, const State &to, const double &diff2, const bool &isLeft);

        void GDIP_Lp_Rp(const AngleInterval& from, const AngleInterval& to, const double &diff2);

        void GDIP_CCp(const State& from, const AngleInterval& to, const double &diff2, bool isLeft);
        void GDIP_CpC(const AngleInterval& from, const State& to, const double &diff2, bool isLeft);

        void GDIP_CCC(const State &from, const State &to, const double &diff2, const bool &isLeft);

        static void Dubins_StateToCircle(const State& from, const Circle& goal, const double& radius, const bool& isLeft,
                                         double& turn, double& straight, double& length);

    };

    std::ostream &operator<<(std::ostream &os, const Dubins &d);

}
