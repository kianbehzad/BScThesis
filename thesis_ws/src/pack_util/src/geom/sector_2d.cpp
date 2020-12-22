// -*-c++-*-

/*!
  \file sector_2d.cpp
  \brief 2D sector region Source File.
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

#include "pack_util/geom/sector_2d.h"

namespace rcsc {

/*-------------------------------------------------------------------*/
/*!

*/
    Sector2D::Sector2D(const Vector2D & c,
                       const double & min_r,
                       const double & max_r,
                       const AngleDeg & start,
                       const AngleDeg & end)
            : M_center(c)
            , M_min_radius(min_r), M_max_radius(max_r)
            , M_angle_left_start(start), M_angle_right_end(end) {
        if (min_r < 0.0) {
            std::cerr << "Sector2D::Sector2D() radius must be positive value."
                      << std::endl;
            M_min_radius = 0.0;
        }
        if (M_min_radius > M_max_radius) {
            std::cerr << "Sector2D::Sector2D(): max radius must be bigger than min radius."
                      << std::endl;
            M_max_radius = M_min_radius;
        }
    }


    const Sector2D &Sector2D::assign(const Vector2D & c,
                     const double & min_r,
                     const double & max_r,
                     const AngleDeg & start,
                     const AngleDeg & end) {
        M_center = c;
        M_min_radius = min_r;
        M_max_radius = max_r;
        M_angle_left_start = start;
        M_angle_right_end = end;

        if (min_r < 0.0) {
            std::cerr << "Sector2D::assign() radius must be positive value."
                      << std::endl;
            M_min_radius = 0.0;
        }
        if (min_r > max_r) {
            std::cerr << "Sector2D::assign() max radius must be bigger than min radius."
                      << std::endl;
            M_max_radius = M_min_radius;
        }

        return *this;
    }


    const double & Sector2D::radiusMin() const {
        return M_min_radius;
    }


    const double & Sector2D::radiusMax() const {
        return M_max_radius;
    }


    const AngleDeg & Sector2D::angleLeftStart() const {
        return M_angle_left_start;
    }


    const AngleDeg & Sector2D::angleRightEnd() const {
        return M_angle_right_end;
    }


    bool Sector2D::contains(const Vector2D & point) const {
        Vector2D rel = point - center();
        double d2 = rel.r2();
        return (M_min_radius * M_min_radius <= d2
                && d2 <= M_max_radius * M_max_radius
                && rel.th().isWithin(M_angle_left_start,
                                     M_angle_right_end));
    }


    double Sector2D::area() const {
        double circle_area
                = (radiusMax() * radiusMax() * M_PI)
                  - (radiusMin() * radiusMin() * M_PI);
        double angle_width
                = (angleRightEnd() - angleLeftStart()).degree();
        if (angle_width < 0.0) {
            angle_width += 360.0;
        }

        circle_area *= (angle_width / 360.0);
        return circle_area;
    }


    double Sector2D::getCircumferenceMin() const {
        double div = (M_angle_right_end - M_angle_left_start).degree();
        if (div < 0.0) {
            div += 360.0;
        }
        return (2.0 * M_min_radius * M_PI) * (div / 360.0);
    }


    double Sector2D::getCircumferenceMax() const {
        double div = (M_angle_right_end - M_angle_left_start).degree();
        if (div < 0.0) {
            div += 360.0;
        }
        return (2.0 * M_max_radius * M_PI) * (div / 360.0);
    }




} // end of namespace
