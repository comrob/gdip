/**
 * @file angleintervaltest.h
 * @author Petr Vana
 * @brief Test merging of multiple angle intervals from special Euclidean space S
 */

#include "gtest/gtest.h"
#include "opendubins/angleinterval.h"

using namespace std;
using namespace opendubins;

namespace angleintervaltest {

    const int COUNT = 10000;

    int ins(vector<AngleInterval>& ints, double angle){
        int ret = 0;
        for(auto& i  : ints){
            if(i.inInterval(angle)){
                ret++;
            }
        }
        return ret;
    }

    TEST (AngleIntervalTest, Merge) {

        const int COUNT_RND = 10;

        for (int i = 0; i < COUNT; ++i) {
            vector<AngleInterval> intervals;

            for(int j = 0; j < 3; ++j){
                auto a = AngleInterval(Point(), myRandom()*2*M_PI, myRandom()*2*M_PI);
                intervals.push_back(a);
            }

            auto merged = AngleInterval::mergeIntervals(intervals);

            for(int j = 0; j < COUNT_RND; ++j){
                double angle = 2 * M_PI * myRandom();

                if(ins(intervals, angle) > 0){
                    EXPECT_EQ(1, ins(merged, angle)) << "Should intersect angle = " << angle;
                }else{
                    EXPECT_EQ(0, ins(merged, angle));
                }
            }
        }
    }

}
