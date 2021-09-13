/**
 * @file point.h
 * @author Petr Vana
 * @brief Point in the 2D plane.
 */


#pragma once

#include "vector.h"

namespace opendubins {

    class Point {
    public:

        double x;
        double y;

        Point() {
            x = 0;
            y = 0;
        }

        Point(double x, double y) {
            this->x = x;
            this->y = y;
        }

        inline double getX() const {
            return x;
        }

        inline double getY() const {
            return y;
        }

        static Point getInvalid(){
            return Point(GDIP_NAN, GDIP_NAN);
        }

        inline bool isValid() const{
            return (!std::isnan(x)) && (!std::isnan(y));
        }

        inline Vector operator-(const Point &r) const {
            return Vector(x - r.x, y - r.y);
        }

        inline Point operator+(const Vector &right) const {
            return Point(x + right.dx, y + right.dy);
        }

        inline Point operator-(const Vector &right) const {
            return Point(x - right.dx, y - right.dy);
        }

        inline Point &operator+=(const Vector &rhs) {
            this->x += rhs.dx;
            this->y += rhs.dy;
            return *this;
        }

        inline Point &operator+=(const Point &rhs) {
            this->x += rhs.x;
            this->y += rhs.y;
            return *this;
        }

        inline Point &operator-=(const Vector &rhs) {
            this->x -= rhs.dx;
            this->y -= rhs.dy;
            return *this;
        }

        inline Point &operator-=(const Point &rhs) {
            this->x -= rhs.x;
            this->y -= rhs.y;
            return *this;
        }

        inline Point operator+(const Point &right) const {
            return Point(x + right.x, y + right.y);
        }

        inline Point operator/(const double div) const {
            if (div != 0)
                return Point(x / div, y / div);
            else
                return Point();
        }

        inline double distance(const Point &to) const {
            return (*this - to).length();
        }

        inline double distanceSquared(const Point &to) const {
            return (*this - to).lengthSquared();
        }

        inline Point interpolate(const Point &p, double &alpha) const {
            double beta = 1 - alpha;
            return Point(beta * x + alpha * p.x, beta * y + alpha * p.y);
        }

        inline bool operator==(const Point &b) const {
            return (x == b.x) && (y == b.y);
        }

        inline size_t hash() const {
            auto h1 = std::hash<double>()(x);
            auto h2 = std::hash<double>()(y);
            return h1 ^ (h2 << 1);
        }
    };

    inline std::ostream &operator<<(std::ostream &os, const Point &d) {
        os << "Point: (" << d.x << ", " << d.y << ")";
        return os;
    }

}
