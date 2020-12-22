// -*-c++-*-

/*!
  \file line_2d.cpp
  \brief 2D straight line class Source File.
*/

/*
 *Copyright:

 Copyright (C) Hidehisa Akiyama

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

#include "pack_util/geom/line_2d.h"

namespace rcsc {

    const double Line2D::EPSILOON = 1.0e-5;
    const double Line2D::ERROR_VALUE = std::numeric_limits< double >::max();


    Line2D::Line2D(const double & a, const double & b, const double & c) : M_a(a), M_b(b), M_c(c) {}


    Line2D::Line2D(const Vector2D & p1, const Vector2D & p2) {
        assign(p1, p2);
    }


    Line2D::Line2D(const Vector2D & origin, const AngleDeg & linedir) {
        assign(origin, linedir);
    }


    const Line2D & Line2D::assign(const Vector2D & p1, const Vector2D & p2) {
        M_a = -(p2.y - p1.y);
        M_b = p2.x - p1.x;
        M_c = -M_a * p1.x - M_b * p1.y;
        return *this;
    }


    const Line2D & Line2D::assign(const Vector2D & origin, const AngleDeg & linedir) {
        M_a = - linedir.sin();
        M_b = linedir.cos();
        M_c = -M_a * origin.x - M_b * origin.y;
        return *this;
    }


    const double & Line2D::a() const {
        return M_a;
    }


    const double & Line2D::getA() const {
        return M_a;
    }


    const double & Line2D::b() const {
        return M_b;
    }


    const double & Line2D::getB() const {
        return M_b;
    }


    const double & Line2D::c() const {
        return M_c;
    }


    const double & Line2D::getC() const {
        return M_c;
    }


    double Line2D::getX(const double & y) const {
        if (std::fabs(M_a) < EPSILOON) {
            return ERROR_VALUE;
        }
        return -(M_b * y + M_c) / M_a;
    }


    double Line2D::getY(const double & x) const {
        if (std::fabs(M_b) < EPSILOON) {
            return ERROR_VALUE;
        }

        return -(M_a * x + M_c) / M_b;
    }


    double Line2D::dist(const Vector2D & p) const {
        return std::fabs((M_a * p.x + M_b * p.y + M_c)
                         / std::sqrt(M_a * M_a + M_b * M_b));
    }


    double Line2D::dist2(const Vector2D & p) const {
        double d = M_a * p.x + M_b * p.y + M_c;
        return (d * d) / (M_a * M_a + M_b * M_b);
    }


    bool Line2D::isParallel(const Line2D & line) const {
        return std::fabs(a() * line.b() - line.a() * b()) < EPSILOON;
    }


    Vector2D Line2D::intersection(const Line2D & line) const {
        return intersection(*this, line);
    }


    Line2D Line2D::perpendicular(const Vector2D & p) const {
        return Line2D(b(), -a(), a() * p.y - b() * p.x);
    }


    Vector2D Line2D::projection(const Vector2D & p) const {
        return intersection(perpendicular(p));
    }


    Vector2D Line2D::intersection(const Line2D & line1, const Line2D & line2) {
        double tmp = line1.a() * line2.b() - line1.b() * line2.a();
        if (std::fabs(tmp) < EPSILOON) {
            return Vector2D::INVALIDATED;
        }

        return Vector2D((line1.b() * line2.c() - line2.b() * line1.c()) / tmp,
                        (line2.a() * line1.c() - line1.a() * line2.c()) / tmp);
    }


    Line2D Line2D::angle_bisector(const Vector2D & origin, const AngleDeg & left, const AngleDeg & right) {
        return Line2D(origin, AngleDeg::bisect(left, right));
    }


    Line2D Line2D::perpendicular_bisector(const Vector2D & p1, const Vector2D & p2) {
        if (std::fabs(p2.x - p1.x) < EPSILOON
            && std::fabs(p2.y - p1.y) < EPSILOON) {
            // input points have same coordiate values.
            std::cerr << "Line2D::perpendicular_bisector."
                      << " ***ERROR*** input points have same coordinate values "
                      << p1 << p2
                      << std::endl;
        }

        double tmp = (p2.x * p2.x - p1.x * p1.x
                      + p2.y * p2.y - p1.y * p1.y) * -0.5 ;
        return Line2D(p2.x - p1.x,
                      p2.y - p1.y,
                      tmp);
    }


} // end of namespace
