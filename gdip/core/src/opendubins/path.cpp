/**
 * @file path.cpp
 * @author Petr Vana
 * @brief Path is an abstract class for Line and Arc which are basic segments in Dubins maneuver.
 */

#include "path.h"

namespace opendubins {

    Path::Path() { }

    Path::~Path() { }

    State Path::getStart() const {
        return getState(0);
    }

    State Path::getEnd() const {
        return getState(getLength());
    }

    State Path::getClosestState(const Point &p) const {
        return getClosestStateAndDistance(p).state;
    }

}
