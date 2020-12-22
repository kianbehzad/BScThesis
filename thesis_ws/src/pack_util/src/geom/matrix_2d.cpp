// -*-c++-*-

/*!
  \file matrix_2d.cpp
  \brief 2D transform matrix class Source File.
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

#include "pack_util/geom/matrix_2d.h"

namespace rcsc {


    Matrix2D::Matrix2D()
            : M_11(1.0)
            , M_12(0.0)
            , M_21(0.0)
            , M_22(1.0)
            , M_dx(0.0)
            , M_dy(0.0) {
    }


    Matrix2D::Matrix2D(const double & m11, const double & m12,
             const double & m21, const double & m22,
             const double & dx, const double & dy)
            : M_11(m11), M_12(m12)
            , M_21(m21), M_22(m22)
            , M_dx(dx),  M_dy(dy) {
    }


    const Matrix2D & Matrix2D::reset() {
        M_11 = M_22 = 1.0;
        M_12 = M_21 = M_dx = M_dy = 0.0;
        return *this;
    }


    const Matrix2D & Matrix2D::assign(const double & m11, const double & m12,
                            const double & m21, const double & m22,
                            const double & dx, const double & dy) {
        M_11 = m11;
        M_12 = m12;
        M_21 = m21;
        M_22 = m22;
        M_dx = dx;
        M_dy = dy;
        return *this;
    }


    Matrix2D Matrix2D::make_translation(const double & dx, const double & dy) {
        return Matrix2D(1.0, 0.0,
                        0.0, 1.0,
                        dx, dy);
    }


    Matrix2D Matrix2D::make_scaling(const double & sx, const double & sy){
        return Matrix2D(sx, 0.0,
                        0.0, sy,
                        0.0, 0.0);
    }


    Matrix2D Matrix2D::make_rotation(const AngleDeg & angle){
        double cosa = angle.cos();
        double sina = angle.sin();
        return Matrix2D(cosa, -sina,
                        sina, cosa,
                        0.0, 0.0);
    }


    const double & Matrix2D::m11() const {
        return M_11;
    }


    const double & Matrix2D::m12() const {
        return M_12;
    }


    const double & Matrix2D::m21() const {
        return M_21;
    }


    const double & Matrix2D::m22() const {
        return M_22;
    }


    const double & Matrix2D::dx() const {
        return M_dx;
    }


    const double & Matrix2D::dy() const {
        return M_dy;
    }


    double Matrix2D::det() const {
        return M_11 * M_22 - M_12 * M_21;
    }


    bool Matrix2D::invertible() const {
        return !(std::fabs(det()) < 0.00000000001);
    }


    Matrix2D Matrix2D::inverted() const {
        double determinant = det();
        if (determinant == 0.0) {
            // never invertible
            return Matrix2D(); // default matrix
        }

        double dinv = 1.0 / determinant;
        return Matrix2D(M_22 * dinv, -M_12 * dinv,
                        -M_21 * dinv, M_11 * dinv,
                        (M_12 * M_dy - M_dx * M_22) * dinv,
                        (M_dx * M_21 - M_11 * M_dy) * dinv);
    }


    const Matrix2D & Matrix2D::translate(const double & dx, const double & dy) {
        // translation matrix
        // T = ( 1, 0, dx )
        //     ( 0, 1, dy )
        //     ( 0, 0,  1 )

        /*
        // this = this * T
        M_dx += M_11*dx + M_12*dy;
        M_dy += M_21*dx + M_22*dy;
        */

        // this = T * this
        // *this = make_translation(dx,dy) * *this;

        M_dx += dx;
        M_dy += dy;
        return *this;
    }


    const Matrix2D & Matrix2D::scale(const double & sx, const double & sy) {
        // scaling matrixa
        // S = ( Sx,  0, 0 )
        //     (  0, Sy, 0 )
        //     (  0,  0, 1 )

        /*
          this = this * S
          *this *= make_scaling(sx,sy)
          M_11 *= sx; M_12 *= sy;
          M_21 *= sx; M_22 *= sy;
        */

        // this = S * this
        // *this = make_scaling(sx,sy) * *this;

        M_11 *= sx;
        M_12 *= sx;
        M_dx *= sx;
        M_21 *= sy;
        M_22 *= sy;
        M_dy *= sy;
        return *this;
    }


    const Matrix2D &Matrix2D::rotate(const AngleDeg & angle) {
        // rotate matrix
        // R = ( cona, -sina, 0 )
        //     ( sina,  cosa, 0 )
        //     (    0,     0, 1 )

        // this = R * this
        // *this = create_rotation(angle) * *this;

        double sina = angle.sin();
        double cosa = angle.cos();

        double tm11 = M_11 * cosa - M_21 * sina;
        double tm12 = M_12 * cosa - M_22 * sina;
        double tm21 = M_11 * sina + M_21 * cosa;
        double tm22 = M_12 * sina + M_22 * cosa;
        double tdx = M_dx * cosa - M_dy * sina;
        double tdy = M_dx * sina + M_dy * cosa;
        M_11 = tm11;
        M_12 = tm12;
        M_dx = tdx;
        M_21 = tm21;
        M_22 = tm22;
        M_dy = tdy;
        return *this;
    }


    const Matrix2D & Matrix2D::operator*=(const Matrix2D & m) {
        double tm11 = M_11 * m.M_11 + M_12 * m.M_21;
        double tm12 = M_11 * m.M_12 + M_12 * m.M_22;
        double tm21 = M_21 * m.M_11 + M_22 * m.M_21;
        double tm22 = M_21 * m.M_12 + M_22 * m.M_22;

        double tdx  = M_11 * m.M_dx + M_12 * m.M_dy + M_dx;
        double tdy =  M_21 * m.M_dx + M_22 * m.M_dy + M_dy;

        M_11 = tm11;
        M_12 = tm12;
        M_21 = tm21;
        M_22 = tm22;
        M_dx = tdx;
        M_dy = tdy;
        return *this;
    }


    Vector2D Matrix2D::transform(const Vector2D & v) const {
        return Vector2D(M_11 * v.x + M_12 * v.y + M_dx,
                        M_21 * v.x + M_22 * v.y + M_dy);
    }


    Vector2D Matrix2D::transform(const double & x, const double & y) const {
        return Vector2D(M_11 * x + M_12 * y + M_dx,
                        M_21 * x + M_22 * y + M_dy);
    }


    void Matrix2D::transform(Vector2D * v) const {
        double tx = M_11 * v->x + M_12 * v->y + M_dx;
        double ty = M_21 * v->x + M_22 * v->y + M_dy;
        v->assign(tx, ty);
    }


    std::ostream & Matrix2D::print(std::ostream & os) const {
        os << M_11 << ' '
           << M_12 << ' '
           << M_21 << ' '
           << M_22 << ' '
           << M_dx << ' '
           << M_dy;
        return os;
    }



} // end of namespace
