// -*-c++-*-

/*!
  \file circle_2d.cpp
  \brief 2D circle region Source File.
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

#include "pack_util/geom/circle_2d.h"
#include "pack_util/geom/triangle_2d.h"
#include "pack_util/geom/ray_2d.h"
#include "pack_util/geom/line_2d.h"
#include "pack_util/geom/segment_2d.h"
#include "pack_util/geom/vector_2d.h"

#include <iostream>
#include <cmath>

// available only this file
namespace {

/*-------------------------------------------------------------------*/
/*!
  \brief get squared value
  \param val input value
  \return squared value
 */
    inline
    double
    SQUARE(const double & val) {
        return val * val;
    }

/*-------------------------------------------------------------------*/
/*!
  \brief solve quadratic fomula
  \param a fomula constant A
  \param b fomula constant B
  \param c fomula constant C
  \param sol1 reference to the result variable
  \param sol2 reference to the result variable
  \return number of solution
 */
    inline
    int
    QUADRATIC_FOMULA(const double & a,
                     const double & b,
                     const double & c,
                     double & sol1,
                     double & sol2) {
        double d = SQUARE(b) - 4.0 * a * c;
        // ignore small noise
        if (std::fabs(d) < 0.001) {
            sol1 = -b / (2.0 * a);
            return 1;
        } else if (d < 0.0) {
            return 0;
        } else {
            d = std::sqrt(d);
            sol1 = (-b + d) / (2.0 * a);
            sol2 = (-b - d) / (2.0 * a);
            return 2;
        }
    }

} // end of namespace


namespace rcsc {


    const double Circle2D::EPSILOON = 1.0e-5;



    Circle2D::Circle2D() : M_center(0.0, 0.0), M_radius(0.0) {}


    Circle2D::Circle2D(const Vector2D & c, const double & r) : M_center(c), M_radius(r) {
        if (r < 0.0) {
            std::cerr << "Circle2D::Circle2D(). radius must be positive value."
                      << std::endl;
            M_radius = 0.0;
        }
    }


    const Circle2D & Circle2D::assign(const Vector2D & c, const double & r) {
        M_center = c;
        M_radius = r;
        if (r < 0.0) {
            std::cerr << "Circle2D::assign(). radius must be positive value."
                      << std::endl;
            M_radius = 0.0;
        }
        return *this;
    }


    bool Circle2D::contains(const Vector2D & point) const {
        return M_center.dist2(point) < M_radius * M_radius;
    }


    const Vector2D & Circle2D::center() const {
        return M_center;
    }


    const double & Circle2D::radius() const {
        return M_radius;
    }


    int Circle2D::intersection(const Line2D & line, Vector2D * sol1, Vector2D * sol2) const {
        if (std::fabs(line.a()) < EPSILOON) {
            if (std::fabs(line.b()) < EPSILOON) {
//            std::cerr << "Circle2D::intersection() illegal line."
//                      << std::endl;
                return 0;
            }

            // Line:    By + C = 0  ---> y = -C/B
            // Circle:  (x - cx)^2 + (y - cy)^2 = r^2
            // --->
            double x1 = 0.0, x2 = 0.0;
            int n_sol
                    = QUADRATIC_FOMULA(1.0,
                                       -2.0 * center().x,
                                       (SQUARE(center().x)
                                        + SQUARE(line.c() / line.b() + center().y)
                                        - SQUARE(radius())),
                                       x1,
                                       x2);

            if (n_sol > 0) {
                double y1 = -line.c() / line.b();

                if (sol1) {
                    sol1->assign(x1, y1);
                }

                if (n_sol > 1 && sol2) {
                    sol2->assign(x2, y1);
                }
            }
            return n_sol;
        } else {
            // include (fabs(l.b()) < EPSILOON) case
            // use line & circle formula
            //   Ax + By + C = 0
            //   (x - cx)^2 + (y - cy)^2 = r^2
            // make y's quadratic formula using these fomula.
            double m = line.b() / line.a();
            double d = line.c() / line.a();

            double a = 1.0 + m * m;
            double b = 2.0 * (-center().y + (d + center().x) * m);
            double c = SQUARE(d + center().x)
                       + SQUARE(center().y)
                       - SQUARE(radius());

            double y1 = 0.0, y2 = 0.0;
            int n_sol = QUADRATIC_FOMULA(a, b, c,
                                         y1, y2);

            if (n_sol > 0 && sol1) {
                sol1->assign(line.getX(y1), y1);
            }

            if (n_sol > 1 && sol2) {
                sol2->assign(line.getX(y2), y2);
            }

            return n_sol;
        }
    }


    int Circle2D::intersection(const Ray2D & ray, Vector2D * sol1, Vector2D * sol2) const {
        Line2D line(ray.origin(), ray.dir());
        Vector2D tsol1, tsol2;

        int n_sol = intersection(line, &tsol1, &tsol2);

        if (n_sol > 1
            && ! ray.inRightDir(tsol2, 1.0)) {
            --n_sol;
        }

        if (n_sol > 0
            && ! ray.inRightDir(tsol1, 1.0)) {
            tsol1 = tsol2; // substituted by second solution
            --n_sol;
        }

        if (n_sol > 0 && sol1) {
            *sol1 = tsol1;
        }

        if (n_sol > 1 && sol2) {
            *sol2 = tsol2;
        }

        return n_sol;
    }


    int Circle2D::intersection(const Circle2D & circle, Vector2D * sol1, Vector2D * sol2) const {
        double rel_x = circle.center().x - this->center().x;
        double rel_y = circle.center().y - this->center().y;

        double center_dist2 = rel_x * rel_x + rel_y * rel_y;
        double center_dist = std::sqrt(center_dist2);

        if (center_dist < std::fabs(this->radius() - circle.radius())
            || this->radius() + circle.radius() < center_dist) {
            return 0;
        }

        //std::cerr << "must exist intersection C1: " << this->center() << this->radius()
        //        << " C2: " << circle.center() << circle.radius()
        //        << std::endl;
        // line that passes through the intersection points
        Line2D line(-2.0 * rel_x,
                    -2.0 * rel_y,
                    circle.center().r2()
                    - circle.radius() * circle.radius()
                    - this->center().r2()
                    + this->radius() * this->radius());

        return this->intersection(line, sol1, sol2);
    }


    int Circle2D::intersection(const Segment2D & seg, Vector2D * sol1, Vector2D * sol2) const {
        Line2D line = seg.line();
        Vector2D tsol1(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
        Vector2D tsol2(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);

        int n_sol = intersection(line, &tsol1, &tsol2);
        if (n_sol > 1) {
            if (seg.contains(tsol1) && seg.contains(tsol2)) {
                *sol1 = tsol1;
                *sol2 = tsol2;
                return 2;
            } else if (seg.contains(tsol1) && !seg.contains(tsol2)) {
                *sol1 = tsol1;
                *sol2 = Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
                return 1;
            } else if (seg.contains(tsol2) && !seg.contains(tsol1)) {
                *sol1 = tsol2;
                *sol2 = Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
                return 1;
            } else {
                *sol1 = Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
                *sol2 = Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
                return 0;
            }
        } else if (n_sol > 0) {
            if (tsol1.valid()) {
                *sol1 = tsol1;
                *sol2 = Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
                return 1;
            } else {
                *sol1 = tsol2;
                *sol2 = Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
                return 1;
            }
        } else {
            *sol1 = Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
            *sol2 = Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
            return 0;
        }
    }


    int Circle2D::intersection(const Segment2D & seg) const {
        Line2D line = seg.line();
        Vector2D tsol1(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
        Vector2D tsol2(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);

        int n_sol = intersection(line, &tsol1, &tsol2);
        if (n_sol > 1) {
            if (seg.contains(tsol1) && seg.contains(tsol2)) return 2;
            else if (seg.contains(tsol1) || seg.contains(tsol2)) return 1;
            else return 0;
        } else if (n_sol > 0) {
            return 1;
        } else {
            return 0;
        }
    }


    int Circle2D::tangent(Vector2D p, Vector2D * sol1, Vector2D * sol2) {
        double s = p.dist2(M_center);
        double r = M_radius * M_radius;
        if (s < r) {
            return 0;
        }
        if (s == r) {
            sol1->assign(p.x, p.y);
            return 1;
        }
        return intersection(Circle2D(p, sqrt(s - r)), sol1, sol2);
    }


    Circle2D Circle2D::circumcircle(const Vector2D & a, const Vector2D & b, const Vector2D & c) {
        Vector2D center = Triangle2D::circumcenter(a, b, c);

        if (! center.valid()) {
            std::cerr << "Circle2D::circumcircle()"
                      << " ***ERROR*** failed to get circumcenter from "
                      << a << b << c
                      << std::endl;
            return Circle2D();
        }

        return Circle2D(center, center.dist(a));
    }



} // end of namespace
