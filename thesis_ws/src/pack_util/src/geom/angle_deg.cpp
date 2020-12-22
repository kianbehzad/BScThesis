// -*-c++-*-

/*!
  \file angle_deg.cpp
  \brief Degree wrapper class Source File.
*/

/*
 *Copyright:

 Copyright (C) 2004 Hidehisa Akiyama

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

#include "pack_util/geom/angle_deg.h"


#ifndef M_PI
//! PI value macro
#define M_PI 3.14159265358979323846
#endif

namespace rcsc {

const double AngleDeg::EPSILOON = 1.0e-5;

const double AngleDeg::DEG2RAD = M_PI / 180.0;
const double AngleDeg::RAD2DEG = 180.0 / M_PI;

/*-------------------------------------------------------------------*/
/*!

 */

    AngleDeg::AngleDeg() : M_degree(0.0) {}


    AngleDeg::AngleDeg(const double & deg) : M_degree(deg) {
        normalize();
    }


    const AngleDeg & AngleDeg::operator=(const double & deg) {
    M_degree = deg;
    return normalize();
    }


    const AngleDeg & AngleDeg::normalize() {
        if (M_degree < -360.0 || 360.0 < M_degree) {
            M_degree = std::fmod(M_degree, 360.0);
        }

        if (M_degree < -180.0) {
            M_degree += 360.0;
        }

        if (M_degree > 180.0) {
            M_degree -= 360.0;
        }

        return *this;
    }

    const double & AngleDeg::degree() const {
        return M_degree;
    }


    double AngleDeg::abs() const {
        return std::fabs(degree());
    }


    double AngleDeg::radian() const {
        return degree() * DEG2RAD;
    }


    AngleDeg AngleDeg::operator-() const {
        return AngleDeg(- degree());
    }


    const AngleDeg & AngleDeg::operator+=(const AngleDeg & angle) {
        M_degree += angle.degree();
        return normalize();
    }


    const AngleDeg & AngleDeg::operator+=(const double & deg) {
        M_degree += deg;
        return normalize();
    }


    const AngleDeg & AngleDeg::operator-=(const AngleDeg & angle) {
        M_degree -= angle.degree();
        return normalize();
    }


    const AngleDeg & AngleDeg::operator-=(const double & deg) {
        M_degree -= deg;
        return normalize();
    }


    const AngleDeg & AngleDeg::operator*=(const double & scalar) {
        M_degree *= scalar;
        return normalize();
    }


    const AngleDeg & AngleDeg::operator/=(const double & scalar) {
        if (std::fabs(scalar) < EPSILOON) {
            return * this;
        }
        M_degree /= scalar;
        return normalize();
    }


    bool AngleDeg::isLeftOf(const AngleDeg & angle) const {
        //return (*this - angle).degree() < 0.0;
        double diff = angle.degree() - this->degree();
        return ((0.0 < diff && diff < 180.0)
                || diff < -180.0);
    }


    bool AngleDeg::isLeftEqualOf(const AngleDeg & angle) const {
        //return (*this - angle).degree() <= 0.0;
        double diff = angle.degree() - this->degree();
        return ((0.0 <= diff && diff < 180.0)
                || diff < -180.0);
    }


    bool AngleDeg::isRightOf(const AngleDeg & angle) const {
        //return (*this - angle).degree() > 0.0;
        double diff = this->degree() - angle.degree();
        return ((0.0 < diff && diff < 180.0)
                || diff < -180.0);
    }


    bool AngleDeg::isRightEqualOf(const AngleDeg & angle) const {
        //return (*this - angle).degree() >= 0.0;
        double diff = this->degree() - angle.degree();
        return ((0.0 <= diff && diff < 180.0)
                || diff < -180.0);
    }


    double AngleDeg::cos() const {
        return std::cos(degree() * DEG2RAD);
    }


    double AngleDeg::sin() const {
        return std::sin(degree() * DEG2RAD);
    }


    double AngleDeg::tan() const {
        return std::tan(degree() * DEG2RAD);
    }


    bool AngleDeg::isWithin(const AngleDeg & left, const AngleDeg & right) const {
        // left to right arc angle is less than 180 degree.
        if (left.isLeftEqualOf(right)) {
            if (left.isLeftEqualOf(*this) && this->isLeftEqualOf(right)) {
                return true;
            }
        }
            // arc angle is more than 180 degree.
        else {
            // check out reverse side
            //if ( *this <= right || left <= *this )
            // == !(right < *this && *this < left)
            if (this->isLeftEqualOf(right) || left.isLeftEqualOf(*this)) {
                return true;
            }
        }
        return false;
    }


    void AngleDeg::sinMinMax(const double & angle_err, double * minsin, double * maxsin) const {
        if (angle_err < 0.0 || 180.0 < angle_err) {
            std::cerr << "AngleDeg::sinMinMax() invalid error range. "
                      << angle_err << std::endl;
            *minsin = -1.0;
            *maxsin = 1.0;
            return;
        }

        double mindir = this->degree() - angle_err;
        double maxdir = this->degree() + angle_err;

        std::vector< double > sol;
        sol.reserve(4);

        if ((mindir < -90.0 && -90.0 < maxdir)
            || (mindir < 270.0 && 270.0 < maxdir)
                ) {
            sol.push_back(-1.0);
        }

        if ((mindir < 90.0 && 90.0 < maxdir)
            || (mindir < -270.0 && -270.0 < maxdir)
                ) {
            sol.push_back(1.0);
        }

        sol.push_back(AngleDeg::sin_deg(mindir));
        sol.push_back(AngleDeg::sin_deg(maxdir));

        *minsin = *std::min_element(sol.begin(), sol.end());
        *maxsin = *std::max_element(sol.begin(), sol.end());
    }


    void AngleDeg::cosMinMax(const double & angle_err, double * mincos, double * maxcos) const {
        if (angle_err < 0.0 || 180.0 < angle_err) {
            std::cerr << "AngleDeg::cosMinMax() invalid error range. "
                      << angle_err << std::endl;
            *mincos = -1.0;
            *maxcos = 1.0;
            return;
        }

        double mindir = this->degree() - angle_err;
        double maxdir = this->degree() + angle_err;

        std::vector< double > sol;
        sol.reserve(4);

        if (mindir < -180.0 && -180.0 < maxdir) {
            sol.push_back(-1.0);
        }

        if (mindir < 0.0 && 0.0 < maxdir) {
            sol.push_back(1.0);
        }

        sol.push_back(AngleDeg::cos_deg(mindir));
        sol.push_back(AngleDeg::cos_deg(maxdir));

        *mincos = *std::min_element(sol.begin(), sol.end());
        *maxcos = *std::max_element(sol.begin(), sol.end());
    }


    AngleDeg AngleDeg::bisect(const AngleDeg & left, const AngleDeg & right) {
        AngleDeg result(left);
        AngleDeg rel(right - left);
        double half_deg = rel.degree() * 0.5;
        result += half_deg;

        if (left.isLeftOf(right)) {
            return result;
        } else {
            return result += 180.0;
        }
    }


    std::ostream & AngleDeg::print(std::ostream & os) const {
        return os << degree();
    }


    std::ostream & AngleDeg::printRound(std::ostream & os, const double & step) const {
        return os << rint(degree() / step) * step;
    }


    AngleIsWithin::result_type AngleDeg::DegreeCmp::operator()(const first_argument_type & lhs,
                           const second_argument_type & rhs) const {
        return lhs.degree() < rhs.degree();
    }



    AngleIsWithin::AngleIsWithin(const AngleDeg & left, const AngleDeg & right)
            : M_left(left)
            , M_right(right) {}



} // end of namespace
