/**
 * @file stateatdistance.h
 * @author Petr Vana
 * @brief Auxiliary structure for returning informations about intersections.
 */

#pragma once

#include "state.h"

namespace opendubins {

    struct StateAtDistance {

        State state;
        double distance;

        StateAtDistance() {
            state = State(Point(NAN, NAN), NAN);
            distance = NAN;
        }

        StateAtDistance(State state, double distance) {
            this->state = state;
            this->distance = distance;
        }

        bool isValid() const {
            return ! std::isnan(distance);
        }

    };

}
