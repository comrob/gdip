/**
 * @file dubinspath.cpp
 * @author Petr Vana
 * @brief This class connects multiple Dubins maneuvers together to enable uniform sampling and collision detection.
 */

#include "dubinspath.h"

namespace opendubins {

    DubinsPath::DubinsPath() {}

    DubinsPath::DubinsPath(std::vector<Dubins> path) {
        this->path = path;
    }

    State DubinsPath::getState(double len) const {
        double sum = 0;
        for (auto & d: path) {
            double diff = len - sum;
            if(diff < d.length){
                return d.getState(diff);
            }
            sum += d.length;
        }
        return path.back().getEnd();
    }

    double DubinsPath::getLength() const{
        double sum = 0;
        for (auto & d: path) {
            sum += d.length;
        }
        return sum;
    }

    bool DubinsPath::intersect(Line line) const{
        for (auto & d: path) {
            if(d.intersectLineBool(line)){
                return true;
            }
        }
        return false;
    }

    StateAtDistance DubinsPath::getClosestStateAndDistance(const Point &p) const {
        StateAtDistance closest;
        // compute distance of all previous maneuvers
        double distance = 0;
        // iterate over all maneuvers
        for(auto & d : path){
            auto state = d.getClosestStateAndDistance(p);
            if(!closest.isValid() || closest.distance > state.distance){
                closest = state;
                closest.distance += distance;
            }
            distance += d.getLength();
        }
        return closest;
    }

    StateAtDistance DubinsPath::intersectionPoint(Line line) const{
        for(auto d : path){
            auto intersection = d.intersectLine(line);
            if(intersection.isValid()){
                return intersection;
            }
        }
        return StateAtDistance();
    }

    std::vector<Point> DubinsPath::interpolate(double step) const {
        std::vector<Point> pnts;

        double speed = 0;

        for(double distance = 0; distance < getLength(); distance += speed){
            pnts.push_back(getState(distance).point);
            speed += step * 0.1;
            if(speed > step){
                speed = step;
            }
        }

        return pnts;
    }
}
