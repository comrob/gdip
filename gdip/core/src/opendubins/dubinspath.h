/**
 * @file dubinspath.h
 * @author Petr Vana
 * @brief This class connects multiple Dubins maneuvers together to enable uniform sampling and collision detection.
 */

#pragma once

#include "path.h"
#include "dubins.h"
#include "stateatdistance.h"

namespace opendubins {

    class DubinsPath : public Path {
    public:

        DubinsPath();
        DubinsPath(std::vector<Dubins> path);

        std::vector<Dubins> path;

        State getState(double len) const;
        double getLength() const;
        bool intersect(Line line) const;

        StateAtDistance getClosestStateAndDistance(const Point &p) const;
        StateAtDistance intersectionPoint(Line line) const;

        std::vector<Point> interpolate(double step) const;

        inline void add(const Dubins & d) {
            path.push_back(d);
        }

    };

    inline std::ostream &operator<<(std::ostream &os, const DubinsPath &d) {
        for(auto & maneuver : d.path) {
            os << "Dubins maneuver " << maneuver.start << " to " << maneuver.end;
        }
        return os;
    }

}


