// -*-c++-*-

/*!
  \file matrix_2d.h
  \brief 2D transform matrix class Header File.
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

#ifndef RCSC_GEOM_MATRIX2D_H
#define RCSC_GEOM_MATRIX2D_H

#include <pack_util/geom/vector_2d.h>
#include <pack_util/geom/angle_deg.h>

#include <iostream>
#include <cmath>

namespace rcsc {

/*!
  \class Matrix2D
  \brief 2D translation matrix class

  ( m11, m12, dx )
  ( m21, m22, dy )
  (   0,   0,  1 )
*/
class Matrix2D {
private:

    double M_11; //!< element (1,1): the horizontal scaling factor.
    double M_12; //!< element (1,2): the vertical shearing factor.
    double M_21; //!< element (2,1): the horizontal shearing factor.
    double M_22; //!< element (2,2): the vertical scaling factor.
    double M_dx; //!< dx: the horizontal translation factor.
    double M_dy; //!< dy: the vertical translation factor.

public:

    /*!
      \brief create an identity matrix
    */
    Matrix2D();

    /*!
      \brief create a matrix with all elements.
      \param m11 the horizontal scaling factor.
      \param m12 the vertical shearing factor.
      \param m21 the horizontal shearing factor.
      \param m22 the vertical scaling factor.
      \param dx the horizontal translation factor.
      \param dy the vertical translation factor.
    */
    Matrix2D(const double & m11, const double & m12,
             const double & m21, const double & m22,
             const double & dx, const double & dy);

    /*!
      \brief reset to the identity matrix
      \return const reference to itself
     */
    const Matrix2D & reset();

    /*!
      \brief set a matrix element with the specified values.
      \param m11 the horizontal scaling factor.
      \param m12 the vertical shearing factor.
      \param m21 the horizontal shearing factor.
      \param m22 the vertical scaling factor.
      \param dx the horizontal translation factor.
      \param dy the vertical translation factor.
      \return const reference to itself
    */
    const Matrix2D & assign(const double & m11, const double & m12,
                      const double & m21, const double & m22,
                      const double & dx, const double & dy);

    /*!
      \brief create the translation matrix.
      \param dx the horizontal translation factor.
      \param dy the vertical translation factor.
      \return new matrix object
     */
    static Matrix2D make_translation(const double & dx, const double & dy);

    /*!
      \brief create the scaling matrix.
      \param sx the horizontal scaling factor.
      \param sy the vertical scaling factor.
      \return new matrix object
     */
    static Matrix2D make_scaling(const double & sx, const double & sy);

    /*!
      \brief create the rotation matrix.
      \param angle the rotation angle
      \return new matrix object
     */
    static Matrix2D make_rotation(const AngleDeg & angle);

    /*!
      \brief get the horizontal scaling factor.
      \return the horizontal scaling factor value.
    */
    const double & m11() const;

    /*!
      \brief get the vertical shearing factor.
      \return the vertical shearing factor value.
    */
    const double & m12() const;

    /*!
      \brief get the horizontal shearing factor.
      \return  the horizontal shearing factor value.
    */
    const double & m21() const;

    /*!
      \brief get the vertical scaling factor.
      \return the vertical scaling factor value.
    */
    const double & m22() const;

    /*!
      \brief get the horizontal translation factor.
      \return the horizontal translation factor value.
    */
    const double & dx() const;

    /*!
      \brief get the vertical translation factor.
      \return the vertical translation factor value.
    */
    const double & dy() const;

    /*!
      \brief get the matrix's determinant.
      \return the determinant value.
     */
    double det() const;

    /*!
      \brief check if this matrix is invertible (is not isingular).
      \return true if this matirix is invertibale.
     */
    bool invertible() const;

    /*!
      \brief get the inverted matrix.
      \return the invverted matrix object.
     */
    Matrix2D inverted() const;

    /*!
      \brief moves the coordinate system.
      \param dx move factor for the x axis.
      \param dy move factor for the y axis.
      \return const reference to itself.

      same as:
        this = make_translation(dx,dy) * this
     */
    const Matrix2D & translate(const double & dx, const double & dy);
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


    /*!
      \brief scales the coordinate system.
      \param sx scaling factor for the x axis.
      \param sy scaling factor for the y axis.
      \return const reference to itself.

      same as:
        this = make_scaling(sx,sy) * this
     */
    const Matrix2D & scale(const double & sx, const double & sy);
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


    /*
    const
    Matrix2D & shear( const double & sh,
                      const double & sv )
      {
          double tm11 = sv * M_21;
          double tm12 = sv * M_22;
          double tm21 = sh * M_11;
          double tm22 = sh * M_12;
          M_11 += tm11; M_12 += tm12;
          M_21 += tm21; M_22 += tm22;
          return *this;
      }
    */

    /*!
      \brief rotates the coordinate system
      \param angle rotation angle
      \return const reference to itself

      same as:
        this = make_rotation(angle) * this
     */
    const Matrix2D & rotate(const AngleDeg & angle);

    /*!
      \brief multiplied by other matrix
      \param m left hand side matrix
      \return const reference to itself
     */
    const Matrix2D & operator*=(const Matrix2D & m);

    /*!
      \brief create transformed vector from input vector with this matrix
      \param v input vector
      \return mapped vector object
     */
    Vector2D transform(const Vector2D & v) const;

    /*!
      \brief create transformed vector from input coordinates with this matrix
      \param x input x-coordinates value
      \param y input y-coordinates value
      \return mapped vector object
     */
    Vector2D transform(const double & x, const double & y) const;

    /*!
      \brief transform input vector with this matrix
      \param v pointer to the input vector.
     */
    void transform(Vector2D * v) const;
#if 0
    Segment2D transform(const Segment2D & s) const {
        return Segment2D(transform(s.a()),
                         transform(s.b()));
    }

    /*
      Line2D transform( const Line2D & l ) const
      {
      }
     */

    Rai2D transform(const Ray2D & r) const {
        return Ray2D(transform(r.origin()),
                     transform(r.origin() + Vector2D::polar2vector(1.0, r.dir())));
    }

    Circle2D transform(const Circle2D & c) const {
        return Circle2D(transform(c.center()),
                        c.radius());
    }

    /*
    Sector2D transform( const Sector2D & s ) const
      {

      }
    */

    Triangle2D transform(const Triangle2D & t) const {
        return Triangle2D(transform(t.a()),
                          transform(t.b()),
                          transform(t.c()));
    }
#endif

    /*!
      \brief put all elemtns to the output stream
      \param os reference to the output stream
      \return reference to the output stream
     */
    std::ostream & print(std::ostream & os) const;

};

} // end of namespace

/*!
  \brief multiplication operator of Matrix x Matrix.
  \param lhs left hand side matrix.
  \param rhs right hand side matrix
  \return result matrix object
 */
inline
const
rcsc::Matrix2D
operator*(const rcsc::Matrix2D & lhs,
          const rcsc::Matrix2D & rhs) {
    return rcsc::Matrix2D(lhs) *= rhs;
}

/*!
  \brief multiplication(transformation) operator of Matrix x Vector.
  \param lhs left hand side matrix.
  \param rhs right hand side vector
  \return result vector object
 */
inline
rcsc::Vector2D
operator*(const rcsc::Matrix2D & lhs,
          const rcsc::Vector2D & rhs) {
    return lhs.transform(rhs);
}

/*!
  \brief output stream operator.
  \param os reference to the output stream.
  \param m value to be output.
  \return reference to the output stream.
 */
inline
std::ostream &
operator<<(std::ostream & os,
           const rcsc::Matrix2D & m) {
    return m.print(os);
}


#endif
