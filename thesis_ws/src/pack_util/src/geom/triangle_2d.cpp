// -*-c++-*-

/*!
  \file triangle_2d.cpp
  \brief 2D triangle class Source File.
*/

/*
 *Copyright:

 Copyright (C) Hidehisa Akiyama

 This code is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 3 of the License, or (at your option) any later version.

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

#include "pack_util/geom/triangle_2d.h"

#include "pack_util/geom/ray_2d.h"

namespace rcsc {


    Triangle2D::Triangle2D(const Vector2D & v1,
               const Vector2D & v2,
               const Vector2D & v3) : M_a(v1), M_b(v2), M_c(v3) {}


    Triangle2D::Triangle2D(const Segment2D & seg, const Vector2D & v) : M_a(seg.origin()), M_b(seg.terminal()), M_c(v) {}


    const Triangle2D & Triangle2D::assign(const Vector2D & v1, const Vector2D & v2, const Vector2D & v3) {
        M_a = v1;
        M_b = v2;
        M_c = v3;
        return *this;
    }


    bool Triangle2D::isValid() const {
        return M_a.isValid()
               && M_b.isValid()
               && M_c.isValid()
               && M_a != M_b
               && M_b != M_c
               && M_a != M_a;
    }


    const Triangle2D & Triangle2D::assign(const Segment2D & seg, const Vector2D & v) {
        M_a = seg.origin();
        M_b = seg.terminal();
        M_c = v;
        return *this;
    }


    const Vector2D & Triangle2D::a() const {
        return M_a;
    }


    const Vector2D & Triangle2D::b() const {
        return M_b;
    }


    const Vector2D & Triangle2D::c() const {
        return M_c;
    }


    double Triangle2D::area() const {
        // outer product == area of parallelogram(Heikou Shihenkei)
        // triangle area is a half of parallelogram area
        return std::fabs((b() - a()).outerProduct(c() - a())) * 0.5;
    }


    double Triangle2D::signedArea() const {
        //return doubleSignedArea() / 2.0;
        return signed_area(a(), b(), c());
    }


    double Triangle2D::signedArea2() const {
        return ((a().x - c().x) * (b().y - c().y)
                + (b().x - c().x) * (c().y - a().y));
    }


    double Triangle2D::doubleSignedArea() const {
        //return ( ( a().x - c().x ) * ( b().y - c().y )
        //         + ( b().x - c().x ) * ( c().y - a().y ) );
        return double_signed_area(a(), b(), c());
    }


    bool Triangle2D::ccw() const {
        return ccw(a(), b(), c());
    }


    bool Triangle2D::contains(const Vector2D & point) const {
        Vector2D rel1(M_a - point);
        Vector2D rel2(M_b - point);
        Vector2D rel3(M_c - point);

        double outer1 = rel1.outerProduct(rel2);
        double outer2 = rel2.outerProduct(rel3);
        double outer3 = rel3.outerProduct(rel1);

        if ((outer1 >= 0.0 && outer2 >= 0.0 && outer3 >= 0.0)
            || (outer1 <= 0.0 && outer2 <= 0.0 && outer3 <= 0.0)) {
            return true;
        }

        return false;
    }


    Vector2D Triangle2D::centroid() const {
        return centroid(a(), b(), c());
    }


    Vector2D Triangle2D::incenter() const {
        return incenter(a(), b(), c());
    }


    Vector2D Triangle2D::circumcenter() const {
        return circumcenter(a(), b(), c());
    }


    Vector2D Triangle2D::orthocenter() const {
        return orthocenter(a(), b(), c());
    }


    int Triangle2D::intersection(const Line2D & line, Vector2D * sol1, Vector2D * sol2) const {
        int n_sol = 0;
        Vector2D tsol[2];

        if (n_sol < 2
            && (tsol[n_sol] = Segment2D(a(), b()).intersection(line)).isValid()) {
            ++n_sol;
        }

        if (n_sol < 2
            && (tsol[n_sol] = Segment2D(b(), c()).intersection(line)).isValid()) {
            ++n_sol;
        }

        if (n_sol < 2
            && (tsol[n_sol] = Segment2D(c(), a()).intersection(line)).isValid()) {
            ++n_sol;
        }

        if (n_sol == 2
            && std::fabs(tsol[0].x - tsol[1].x) < 1.0e-5
            && std::fabs(tsol[0].y - tsol[1].y) < 1.0e-5) {
            n_sol = 1;
        }

        if (n_sol > 0
            && sol1) {
            *sol1 = tsol[0];
        }

        if (n_sol > 1
            && sol2) {
            *sol2 = tsol[1];
        }

        return n_sol;
    }


    int Triangle2D::intersection(const Ray2D & ray, Vector2D * sol1, Vector2D * sol2) const {
        Vector2D tsol1, tsol2;
        int n_sol = intersection(ray.line(), &tsol1, &tsol2);

        if (n_sol > 1
            && ! ray.inRightDir(tsol2, 1.0)) {
            --n_sol;
        }

        if (n_sol > 0
            && ! ray.inRightDir(tsol1, 1.0)) {
            tsol1 = tsol2;
            --n_sol;
        }

        if (n_sol > 0
            && sol1) {
            *sol1 = tsol1;
        }

        if (n_sol > 1
            && sol2) {
            *sol2 = tsol2;
        }

        return n_sol;
    }


    int Triangle2D::intersection(const Segment2D & segment, Vector2D * sol1, Vector2D * sol2) const {
        Vector2D tsol1, tsol2;
        int n_sol = intersection(segment.line(), &tsol1, &tsol2);

        if (n_sol > 1
            && ! segment.contains(tsol2)) {
            --n_sol;
        }

        if (n_sol > 0
            && ! segment.contains(tsol1)) {
            tsol1 = tsol2;
            --n_sol;
        }

        if (n_sol > 0
            && sol1) {
            *sol1 = tsol1;
        }

        if (n_sol > 1
            && sol2) {
            *sol2 = tsol2;
        }

        return n_sol;
    }


    double Triangle2D::double_signed_area(const Vector2D & a, const Vector2D & b, const Vector2D & c) {
        return ((a.x - c.x) * (b.y - c.y)
                + (b.x - c.x) * (c.y - a.y));
    }


    double Triangle2D::signed_area(const Vector2D & a, const Vector2D & b, const Vector2D & c) {
        return double_signed_area(a, b, c) * 0.5;
    }


    bool Triangle2D::ccw(const Vector2D & a, const Vector2D & b, const Vector2D & c) {
        return double_signed_area(a, b, c) > 0.0;
    }


    Vector2D Triangle2D::centroid(const Vector2D & a, const Vector2D & b, const Vector2D & c) {
        Vector2D g(a);
        g += b;
        g += c;
        g /= 3.0;
        return g;
        //return Vector2D( a ).add( b ).add( c ) /= 3.0;
    }


    Vector2D Triangle2D::incenter(const Vector2D & a, const Vector2D & b, const Vector2D & c) {
        Vector2D ab = b - a;
        Vector2D ac = c - a;
        Line2D bisect_a(a,
                        AngleDeg::bisect(ab.th(), ac.th()));

        Vector2D ba = a - b;
        Vector2D bc = c - b;
        Line2D bisect_b(b,
                        AngleDeg::bisect(ba.th(), bc.th()));

        return bisect_a.intersection(bisect_b);
    }


    Vector2D Triangle2D::circumcenter(const Vector2D & a, const Vector2D & b, const Vector2D & c) {
        Line2D perpendicular_ab
                = Line2D::perpendicular_bisector(a, b);

        Line2D perpendicular_bc
                = Line2D::perpendicular_bisector(b, c);

        return perpendicular_ab.intersection(perpendicular_bc);

#if 0
        // Following algorithm seems faster than above method.
    // However, result is as:
    //   above method     10000000times 730 [ms]
    //   following method 10000000times 934 [ms]
    // So, I choose above method.

    ////////////////////////////////////////////////////////////////
    // Q : curcumcenter
    // M : center of AB
    // N : center of AC
    // s, t : parameter
    // <,> : inner product operator
    // S : area of triangle
    // a = |BC|, b = |CA|, c = |AB|

    // AQ = s*AB + t*AC

    // <MQ, AB> = <AQ - AM, AB>
    //          = <s*AB + t*AC - AB/2, AB >
    //          = <(s-1/2)*AB^2 + tAB, AC>
    //          = (s-1/2)*c^2 + t*b*c*cosA
    //          = 0
    // <NQ, AC> = s*b*c*cosA + (t-1/2)*b^2 = 0

    // c^2 * s + (b*c*cosA)*t = c^2 / 2
    // (b*c*cosA)*s + b^2 * t = b^2 / 2

    // s = b^2 * (c^2 + a^2 - b^2) / (16S^2)
    // t = c^2 * (a^2 + b^2 - c^2) / (16S^2)

    // AQ = {b^2 * (c^2 + a^2 - b^2) * AB + c^2 * (a^2 + b^2 - c^2)) * AC} /(16S^2)

    Vector2D ab = b - a;
    Vector2D ac = c - a;

    double tmp = ab.outerProduct(ac);
    if (std::fabs(tmp) < 0.001) {
        // The area of parallelogram is 0.
        std::cerr << "Triangle2D::getCircumCenter()"
                  << " ***ERROR*** at least, two vertex points have same coordiante"
                  << std::endl;
        return Vector2D(Vector2D::INVALID);
    }

    double inv = 0.5 / tmp;
    double ab_len2 = ab.r2();
    double ac_len2 = ac.r2();
    double xcc = inv * (ab_len2 * ac.y - ac_len2 * ab.y);
    double ycc = inv * (ab.x * ac_len2 - ac.x * ab_len2);
    // circle radius = xcc*xcc + ycc*ycc
    return Vector2D(a.x + xcc, a.y + ycc);
#endif
    }


    Vector2D Triangle2D::orthocenter(const Vector2D & a, const Vector2D & b, const Vector2D & c) {
        Line2D perpend_a = Line2D(b, c).perpendicular(a);
        Line2D perpend_b = Line2D(c, a).perpendicular(b);

        return perpend_a.intersection(perpend_b);
    }


    bool Triangle2D::contains(const Vector2D & a, const Vector2D & b, const Vector2D & c, const Vector2D & point) {
        Vector2D rel1(a - point);
        Vector2D rel2(b - point);
        Vector2D rel3(c - point);

        double outer1 = rel1.outerProduct(rel2);
        double outer2 = rel2.outerProduct(rel3);
        double outer3 = rel3.outerProduct(rel1);

        if ((outer1 >= 0.0 && outer2 >= 0.0 && outer3 >= 0.0)
            || (outer1 <= 0.0 && outer2 <= 0.0 && outer3 <= 0.0)) {
            return true;
        }

        return false;
    }



} // end of namespace
