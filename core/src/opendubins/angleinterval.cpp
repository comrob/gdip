/**
 * @file angleinterval.cpp
 * @author Petr Vana
 * @brief Interval of heading angles utilized in both DIP and GDIP formulation
 */

#include "angleinterval.h"

using namespace opendubins;
using namespace std;

namespace opendubins {

    vector<AngleInterval> AngleInterval::mergeIntervals(vector<AngleInterval> input){
        decltype(input) merged;
        decltype(input) twice;
        decltype(input) filtered;

        if(input.size() > 0) {
            Point center = input[0].point;

            // sort intervals
            sort(input.begin(), input.end());

            // make second copy of intervals from [0, 2PI] to [2PI, 4PI]
            twice = input;
            for (auto i : input) {
                i.rightDir += 2 * M_PI;
                twice.push_back(i);
            }

            // merge intervals in interval [0, 4PI]
            // last interval can end behind 4PI (but no problem here)
            double from = twice[0].rightDir;
            double to = from + twice[0].diff;
            for (int i = 1; i < twice.size(); ++i) {
                const auto &it = twice[i];
                if (it.rightDir <= to) {
                    to = max(to, (it.rightDir + it.diff));
                } else {
                    // cannot use constructor directly - a normalization is included
                    AngleInterval ai;
                    ai.rightDir = from;
                    ai.diff = to - from;
                    merged.push_back(ai);
                    from = it.rightDir;
                    to = it.rightDir + it.diff;
                }
            }
            // cannot use constructor directly - a normalization is included
            AngleInterval ai;
            ai.rightDir = from;
            ai.diff = to - from;
            merged.push_back(ai);

            // filter interval and reduce them to interval [2PI, 4PI]
            for(auto i : merged){
                const double M = 2*M_PI;
                auto a = i.rightDir - M;
                auto b = a + i.diff;
                if(a < M && b > 0){
                    a = max(a, 0.0);
                    b = min(b, M);
                    filtered.push_back(AngleInterval(center, a, b-a));
                }
            }
        }

        return filtered;
    }

    ostream& operator<<(ostream &os, const AngleInterval &d) {
        os << "AngleInterval: (" << d.rightDir << "; " << (d.rightDir + d.diff) << "), point " << d.point;
        return os;
    }

}