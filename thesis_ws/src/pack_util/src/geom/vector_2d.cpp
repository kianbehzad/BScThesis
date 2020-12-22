// -*-c++-*-

/*!
  \file vector_2d.cpp
  \brief 2D vector class Source File.
*/

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "pack_util/geom/vector_2d.h"
#include "pack_util/geom/segment_2d.h"
#include "pack_util/geom/circle_2d.h"


int sign(double d1) {
    return (d1 > 0) ? 1 : -1;
}

double max(double d1, double d2) {
    return (d1 > d2) ? d1 : d2;
}

double min(double d1, double d2) {
    return (d1 < d2) ? d1 : d2;
}


namespace rcsc {
    Vector2D intersect_ellipse_dir(Vector2D dir, Vector2D center, double a, double b, double e) {
        Vector2D d = dir;
        d.x = d.x * b / a;
        Segment2D s = Segment2D(center + Vector2D(0.0, e), center + dir * 2.0 * b + Vector2D(0.0, e));
        Vector2D sol1, sol2;
        int n = Circle2D(center, b).intersection(s, &sol1, &sol2);
        if (sol1.valid()) {
            sol1.x = (sol1.x - center.x) * a / b + center.x;
        }
        return sol1;
    }


    bool intersect_ellipse_line(Vector2D point1, Vector2D point2, Vector2D center, double _a, double _b, Vector2D * sol1, Vector2D * sol2) {
        double h = center.x;
        double k = center.y;
        double a = _a;
        double b = _b;
        double x1 = point1.x;
        double y1 = point1.y;
        double x2 = point2.x;
        double y2 = point2.y;
        double xi1, xi2, yi1, yi2;

        float aa, bb, cc, m;
        //
        if (x1 != x2) {
            m = (y2 - y1) / (x2 - x1);
            float c = y1 - m * x1;
            //
            aa = b * b + a * a * m * m;
            bb = 2 * a * a * c * m - 2 * a * a * k * m - 2 * h * b * b;
            cc = b * b * h * h + a * a * c * c - 2 * a * a * k * c + a * a * k * k - a * a * b * b;
        } else {
            //
            // vertical line case
            //
            aa = a * a;
            bb = -2.0 * k * a * a;
            cc = -a * a * b * b + b * b * (x1 - h) * (x1 - h);
        }

        float d = bb * bb - 4 * aa * cc;
        //
        // intersection points : (xi1,yi1) and (xi2,yi2)
        //
        if (d > 0.0) {
            if (x1 != x2) {
                xi1 = (-bb + sqrt(d)) / (2 * aa);
                xi2 = (-bb - sqrt(d)) / (2 * aa);
                yi1 = y1 + m * (xi1 - x1);
                yi2 = y1 + m * (xi2 - x1);
            } else {
                yi1 = (-bb + sqrt(d)) / (2 * aa);
                yi2 = (-bb - sqrt(d)) / (2 * aa);
                xi1 = x1;
                xi2 = x1;
            }
        } else {
            return false; // no intersections
        }
        sol1->x = xi1;
        sol1->y = yi1;
        sol2->x = xi2;
        sol2->y = yi2;
        return true;
    }

}


namespace rcsc {

    //const double Vector2D::ERROR_VALUE = std::numeric_limits< double >::max();
    const double Vector2D::ERROR_VALUE = 5000.0;
    const Vector2D Vector2D::INVALIDATED(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);


    Vector2D::Vector2D() : x(ERROR_VALUE), y(ERROR_VALUE) {}


    Vector2D::Vector2D(const double & xx, const double & yy) : x(xx), y(yy) {}


    Vector2D::Vector2D(const pack_msgs::msg::Vector2D& vec) : x(vec.x), y(vec.y) {}


    pack_msgs::msg::Vector2D Vector2D::toParsianMessage() const {
        pack_msgs::msg::Vector2D vec;
        vec.x = this->x;
        vec.y = this->y;
        return vec;
    }


    bool Vector2D::valid() const {
        return (fabs(x - ERROR_VALUE) > 1e-2 && fabs(y - ERROR_VALUE) > 1e-2);
    }


    bool Vector2D::isValid() const {
        return valid();
    }


    bool Vector2D::equalsWeakly(const Vector2D & other) const {
        //return dist2( other ) < EPSILON * EPSILON;
        return std::fabs(this->x - other.x) < 0.0001
               && std::fabs(this->y - other.y) < 0.0001;
    }


    const Vector2D & Vector2D::assign(const double & xx, const double & yy) {
        x = xx;
        y = yy;
        return *this;
    }


    const Vector2D & Vector2D::setPolar(const double & radius, const AngleDeg & dir) {
        x = radius * dir.cos();
        y = radius * dir.sin();
        return *this;
    }


    const Vector2D & Vector2D::invalidate() {
        x = ERROR_VALUE;
        y = ERROR_VALUE;
        return *this;
    }


    double Vector2D::r2() const {
        return x * x + y * y;
    }


    double Vector2D::r() const {
        //return std::hypot( x, y );
        return std::sqrt(r2());
    }


    double Vector2D::length() const {
        return r();
    }


    AngleDeg Vector2D::th() const {
        return AngleDeg(AngleDeg::atan2_deg(y, x));
    }


    AngleDeg Vector2D::dir() const {
        return th();
    }


    Vector2D Vector2D::abs() const {
        return Vector2D(std::fabs(x), std::fabs(y));
    }


    double Vector2D::absX() const {
        return std::fabs(x);
    }


    double Vector2D::absY() const {
        return std::fabs(y);
    }


    const Vector2D & Vector2D::add(const double & xx, const double & yy) {
        x += xx;
        y += yy;
        return *this;
    }


    const Vector2D & Vector2D::operator+() const {
        return *this;
    }


    Vector2D Vector2D::operator-() const {
        return Vector2D(-x, -y);
    }


    const Vector2D & Vector2D::operator+=(const Vector2D & v) {
        x += v.x;
        y += v.y;
        return *this;
    }


    const Vector2D & Vector2D::operator-=(const Vector2D & v) {
        x -= v.x;
        y -= v.y;
        return *this;
    }



    const Vector2D & Vector2D::operator*=(const double & scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }


    const Vector2D & Vector2D::operator/=(const double & scalar) {
        if (scalar != 0) {
            x /= scalar;
            y /= scalar;
        }
        return *this;
    }


    double &Vector2D::operator [](int i) {
        i %= 2;
        if (i == 0) {
            return x;
        } else {
            return y;
        }
    }


    double Vector2D::dist2(const Vector2D & p) const {
        //return ( Vector2D( *this ) -= p ).r2();
        return (std::pow(this->x - p.x, 2.0)
                + std::pow(this->y - p.y, 2.0));
    }


    double Vector2D::dist(const Vector2D & p) const {
        //return std::hypot( this->x - p.x,
        //                   this->y - p.y );
        return std::sqrt(dist2(p));
    }


    const Vector2D & Vector2D::setLength(const double & len) {
        double mag = this->r();
        if (mag == 0) {
            return *this;
        }
        return ((*this) *= (len / mag));
    }


    Vector2D Vector2D::setLengthVector(const double & len) const {
        return Vector2D(*this).setLength(len);
    }


    const Vector2D & Vector2D::normalize() {
        return setLength(1.0);
    }


    Vector2D Vector2D::norm() const {
        return Vector2D(*this).normalize();
    }


    const Vector2D & Vector2D::rotate(const double & deg) {
        double radius = this->r();
        double rotated_angle = this->th().degree();
        rotated_angle += deg;
        rotated_angle *= AngleDeg::DEG2RAD;
        this->x = radius * std::cos(rotated_angle);
        this->y = radius * std::sin(rotated_angle);
        return *this;
    }


    const Vector2D & Vector2D::rotate(const AngleDeg & angle) {
        return rotate(angle.degree());
    }


    Vector2D Vector2D::rotatedVector(const double & deg) const {
        return Vector2D(*this).rotate(deg);
    }


    Vector2D Vector2D::rotatedVector(const AngleDeg & angle) const {
        return Vector2D(*this).rotate(angle.degree());
    }


    const Vector2D & Vector2D::setDir(const AngleDeg & dir) {
        double radius = this->r();
        x = radius * dir.cos();
        y = radius * dir.sin();
        return *this;
    }


    double Vector2D::innerProduct(const Vector2D & v) const {
        return this->x * v.x + this->y * v.y;
        // ==  |this| * |v| * (*this - v).th().cos()
    }


    double Vector2D::outerProduct(const Vector2D & v) const {
        /*---------------------*
         * assume virtual 3D environment.
         * calculate Z-coordinate of outer product in right hand orientation.
         * For the time being, Input Vector's Z-coordinate is set to ZERO.
         *---------------------*/
        // Normal 3D outer product
        //   xn = this->y * v.z - this->z * v.y;
        //   yn = this->z * v.x - this->x * v.z;
        // # zn = this->x * v.y - this->y * v.x;
        return this->x * v.y - this->y * v.x;
        // == |this| * |v| * (*this - v).th().sin()
    }


    std::ostream & Vector2D::print(std::ostream & os) const {
        os << "(" << x << ", " << y << ")";
        return os;
    }


    std::ostream & Vector2D::printRound(std::ostream & os, const double & prec) const {
        os << "("  << rint(x / prec) * prec
           << ", " << rint(y / prec) * prec << ")";
        return os;
    }



    AngleIsWithin::result_type Vector2D::XCmp::operator()(const first_argument_type & lhs, const second_argument_type & rhs) const {
        return lhs.x < rhs.x;
    }


    AngleIsWithin::result_type Vector2D::YCmp::operator()(const first_argument_type & lhs, const second_argument_type & rhs) const {
        return lhs.y < rhs.y;
    }


    AngleIsWithin::result_type Vector2D::AbsXCmp::operator()(const first_argument_type & lhs, const second_argument_type & rhs) const {
        return lhs.absX() < rhs.absX();
    }


    AngleIsWithin::result_type Vector2D::AbsYCmp::operator()(const first_argument_type & lhs, const second_argument_type & rhs) const {
        return lhs.absY() < rhs.absY();
    }


    AngleIsWithin::result_type Vector2D::XYCmp::operator()(const first_argument_type & lhs, const second_argument_type & rhs) const {
        return (lhs.x < rhs.x
                ? true
                : lhs.x > rhs.x
                  ? false
                  : lhs.y < rhs.y);
        // if ( lhs.x < rhs.x ) return true;
        // else if ( lhs.x > rhs.x ) return false;
        // else return lhs.y < rhs.y;
    }


    AngleIsWithin::result_type Vector2D::YXCmp::operator()(const first_argument_type & lhs,
                           const second_argument_type & rhs) const {
        return (lhs.y < rhs.y
                || (lhs.y == rhs.y && lhs.x < rhs.x));
    }




} // end of namespace
