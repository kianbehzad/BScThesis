// -*-c++-*-

/*!
  \file vector_2d.h
  \brief 2d vector class Header File.
*/

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA, Hiroki Shimora

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

#ifndef RCSC_GEOM_VECTOR2D_H
#define RCSC_GEOM_VECTOR2D_H

#include <pack_util/geom/angle_deg.h>
#include <pack_msgs/msg/vector2_d.hpp>

#include <functional>
#include <iostream>
#include <cmath>

namespace rcsc {

/*!
  \class Vector2D
  \brief 2D point vector class
*/
class Vector2D {
    // : public boost::addable< Vector2D >
    // , public boost::subtractable< Vector2D >
    // , public multipliable2< Vector2D, double >
    // , public dividable2< Vector2D, double >

public:

    //! constant error value for XY (= std::numeric_limits< doulble >::max()).
    static const double ERROR_VALUE;

    //! invalidated value vector
    static const Vector2D INVALIDATED;

    double x; //!< X coordinate
    double y; //!< Y coordinate

    /*!
      \brief default constructor.
    */
    Vector2D();

    /*!
      \brief create Vector with XY value directly.
      \param xx assigned x value
      \param yy assigned y value
    */
    Vector2D(const double & xx, const double & yy);

    /*!
      \brief create Vector with ros message.
      \param vec assigned to this
    */
    Vector2D(const pack_msgs::msg::Vector2D& vec);

    /*!
        \brief convert vector2D to ros message.
        \return pack_message vector2D
    */
    pack_msgs::msg::Vector2D toParsianMessage() const;

    /*!
      \brief check if this vector has validate values.
      \return true if value is validate.
    */
    bool valid() const;

    bool isValid() const;


    /*!
      \brief check if this vector is weakly same as given vector.
      \param other compared vector.
      \return true if weakly same, otherwise false.
    */
    bool equalsWeakly(const Vector2D & other) const;

    //     /*!
    //       \brief type conversion operator. alias of valid().
    //       \return true if value is validate.
    //      */
    //     operator bool() const
    //       {
    //           return valid();
    //       }

    /*!
      \brief assign XY value directly.
      \param xx assigned x value
      \param yy assigned y value
      \return const reference to itself
     */
    const
    Vector2D & assign(const double & xx, const double & yy);

    /*!
      \brief assign XY value from POLAR value.
      \param radius vector's radius
      \param dir vector's angle
      \return const reference to itself
     */
    const Vector2D & setPolar(const double & radius, const AngleDeg & dir);

    const Vector2D & invalidate();

    /*!
      \brief get the squared length of vector.
      \return squared length
     */
    double r2() const;

    /*!
      \brief get the length of vector.
      \return length
     */
    double r() const;

    /*!
      \brief get the length of vector.
      \return length
     */
    double length() const;

    /*!
      \brief get the angle of vector.
      \return angle
     */
    AngleDeg th() const;

    /*!
      \brief get the angle of vector.
      \return angle
     */
    AngleDeg dir() const;

    /*!
      \brief get new vector that XY values were set to absolute value.
      \return new vector that all values are absolute.
     */
    Vector2D abs() const;

    /*!
      \brief get absolute x value
      \return absolute x value
     */
    double absX() const;

    /*!
      \brief get absolute y value
      \return absolute y value
     */
    double absY() const;
    /*!
      \brief add XY values respectively.
      \param xx added x value
      \param yy added y value
      \return const reference to itself
     */
    const Vector2D & add(const double & xx, const double & yy);

    /*!
      \brief return this vector
      \return const reference of this vector
     */
    const Vector2D & operator+() const;

    /*!
      \brief create reversed vector
      \return new vector that XY values are reversed.
     */
    Vector2D operator-() const;

    /*!
      \brief add vector to itself
      \param v added vector
      \return const reference to itself
     */
    const Vector2D & operator+=(const Vector2D & v);

    /*!
      \brief subtract vector to itself
      \param v subtract argument
      \return const reference to itself
     */
    const Vector2D & operator-=(const Vector2D & v);

    /*!
      \brief multiplied by 'scalar'
      \param scalar multiplication argument
      \return const reference to itself
     */
    const Vector2D & operator*=(const double & scalar);

    /*!
      \brief divided by 'scalar'.
      \param scalar division argument
      \return const reference to itself
     */
    const Vector2D & operator/=(const double & scalar);

    /*!
      \brief get a single coordinate.
      \param i the number of coordinate that we want
      \return the wanted coordinate
    */
    double &operator [](int i);

    /*!
      \brief get the squared distance from this to 'p'.
      \param p target point
      \return squared distance to 'p'
    */
    double dist2(const Vector2D & p) const;

    /*!
      \brief get the distance from this to 'p'.
      \param p target point
      \return distance to 'p'
    */
    double dist(const Vector2D & p) const;

    /*!
      \brief set vector length to 'len'.
      \param len new length to be set
      \return const reference to itself
    */
    const Vector2D & setLength(const double & len);

    /*!
      \brief get new vector that the length is set to 'len'
      \param len new length to be set
      \return new vector that the length is set to 'len'
    */
    Vector2D setLengthVector(const double & len) const;


    /*!
      \brief normalize vector. length is set to 1.0.
      \return const reference to itself
    */
    const Vector2D & normalize();

    /*!
      \brief get new normalized vector that the length is set to 1.0
      but angle is same
      \return new normalized vector
    */
    Vector2D norm() const;

    /*!
      \brief rotete this vector with 'deg'
      \param deg rotated angle by double type
      \return const reference to itself
     */
    const Vector2D & rotate(const double & deg);

    /*!
      \brief rotate this vector with 'angle'.
      \param angle rotated angle
      \return const reference to itself
     */
    const Vector2D & rotate(const AngleDeg & angle);

    /*!
      \brief get new vector that is rotated by 'deg'.
      \param deg rotated angle. double type.
      \return new vector rotated by 'deg'
     */
    Vector2D rotatedVector(const double & deg) const;

    /*!
      \brief get new vector that is rotated by 'angle'.
      \param angle rotated angle.
      \return new vector rotated by 'angle'
     */
    Vector2D rotatedVector(const AngleDeg & angle) const;

    /*!
      \brief set vector's angle to 'angle'
      \param dir new angle to be set
      \return const reference to itself
     */
    const Vector2D & setDir(const AngleDeg & dir);

    /*!
      \brief get inner(dot) product with 'v'.
      \param v target vector
      \return value of inner product
    */
    double innerProduct(const Vector2D & v) const;

    AngleDeg angleWith(const Vector2D & v) const {
        double d = (length() * v.length());
        if (d == 0.0) {
            return 0;
        }
        return AngleDeg(acos((this->x * v.x + this->y * v.y) / d) * 180.0 / M_PI);
    }

    inline
    static AngleDeg angleOf(const Vector2D& A, const Vector2D& O, const Vector2D& B) {
        Vector2D a1(A.x - O.x, A.y - O.y);
        Vector2D a2(B.x - O.x, B.y - O.y);
        return AngleDeg(fabs(AngleDeg::normalize_angle(
                                 ((a1).th().degree() - (a2).th().degree())))
                       );
    }

    /*!
      \brief get virtal outer(cross) product with 'v'.
      \param v target vector
      \return value of outer product
    */
    double outerProduct(const Vector2D & v) const;
    /*---------------------*
     * assume virtual 3D environment.
     * calculate Z-coordinate of outer product in right hand orientation.
     * For the time being, Input Vector's Z-coordinate is set to ZERO.
     *---------------------*/
    // Normal 3D outer product
    //   xn = this->y * v.z - this->z * v.y;
    //   yn = this->z * v.x - this->x * v.z;
    // # zn = this->x * v.y - this->y * v.x;
    //////////////////////////////////////////////
    // static utility

    /*!
      \brief get new Vector created by POLAR value.
      \param mag length of vector
      \param theta angle of vector
      \return new vector object
    */
    inline
    static
    Vector2D polar2vector(const double & mag,
                          const AngleDeg & theta) {
        return Vector2D(mag * theta.cos(), mag * theta.sin());
    }

    /*!
      \brief get new Vector created by POLAR value.
      \param mag length of vector
      \param theta angle of vector
      \return new vector object
    */
    inline
    static
    Vector2D from_polar(const double & mag,
                        const AngleDeg & theta) {
        return Vector2D(mag * theta.cos(), mag * theta.sin());
    }

    /*!
      \brief get inner(dot) product for v1 and v2.
      \param v1 input 1
      \param v2 input 2
      \return value of inner product
    */
    inline
    static
    double inner_product(const Vector2D & v1,
                         const Vector2D & v2) {
        return v1.innerProduct(v2);
    }

    /*!
      \brief get outer(cross) product for v1 and v2.
      \param v1 input 1
      \param v2 input 2
      \return value of outer product
    */
    inline
    static
    double outer_product(const Vector2D & v1,
                         const Vector2D & v2) {
        return v1.outerProduct(v2);
    }

    /*!
      \brief static utility. calculate angle between two points and the X axis
      \param pointStart start point
      \param pointTarget target point
      \return degree of angle between two points
    */
    inline
    static
    double dirTo_deg(const Vector2D &firstPoint ,
                     const Vector2D &targetPoint) {
        return AngleDeg::atan2_deg(targetPoint.y - firstPoint.y,
                                   targetPoint.x - firstPoint.x);
    }

    inline
    static
    Vector2D unitVector(const AngleDeg &angle) {
        return Vector2D(angle.cos(), angle.sin());
    }

    inline
    static
    AngleDeg angleBetween(const Vector2D& v1, const Vector2D& v2) {
        return AngleDeg::normalize_angle(v2.th().degree() - v1.th().degree());
    }

    /*!
      \brief static utility. calculate angle between two vectors
      \param v1 first vector
      \param v2 decond vector
      \param return_acute_angle decides wich angle to be returned. two angles can be found between two
             vectors: acute_angle(narrower) or obtuse_angle(broader)
      \return degree of angle between two vectors
    */
    inline
    static
    AngleDeg angleBetween_customized(const Vector2D& v1, const Vector2D& v2, bool return_acute_angle = true) {
        double delta_theta = AngleDeg::normalize_angle(v2.th().degree() - v1.th().degree());
        int sign = delta_theta >= 0 ? +1 : -1;
        double compliment_delta_theta = delta_theta - 360*sign;

        double acute_delta_theta = std::min(fabs(delta_theta), fabs(compliment_delta_theta)) == fabs(delta_theta) ? delta_theta : compliment_delta_theta;
        double obtuse_delta_theta = std::min(fabs(delta_theta), fabs(compliment_delta_theta)) == fabs(delta_theta) ? compliment_delta_theta : delta_theta;

        return return_acute_angle ? acute_delta_theta : obtuse_delta_theta;
    }

    //////////////////////////////////////////////
    // stream utility

    /*!
      \brief output XY values to ostream.
      \param os reference to ostream
      \return reference to ostream
    */
    std::ostream & print(std::ostream & os) const;

    /*!
      \brief output rounded XY values to ostream.
      \param os reference to ostream
      \param prec precision of output value
      \return reference to ostream
    */
    std::ostream & printRound(std::ostream & os, const double & prec = 0.1) const;

    //////////////////////////////////////////////
    // functors for comparison

    /*!
      \class XCmp
      \brief comparison predicate for X value.
    */
    class XCmp
        : public std::binary_function< Vector2D, Vector2D, bool > {
    public:
        //! functional operator
        result_type operator()(const first_argument_type & lhs, const second_argument_type & rhs) const;
    };

    /*!
      \class YCmp
      \brief comparison predicate for Y value.
    */
    class YCmp
        : public std::binary_function< Vector2D, Vector2D, bool > {
    public:
        //! functional operator
        result_type operator()(const first_argument_type & lhs, const second_argument_type & rhs) const;
    };

    /*!
      \class AbsXCmp
      \brief comparison predicate for absolute X value.
    */
    class AbsXCmp
        : public std::binary_function< Vector2D, Vector2D, bool > {
    public:
        //! functional operator
        result_type operator()(const first_argument_type & lhs, const second_argument_type & rhs) const;

    };

    /*!
      \class AbsYCmp
      \brief comparison predicate for absolute Y value.
    */
    class AbsYCmp
        : public std::binary_function< Vector2D, Vector2D, bool > {
    public:
        //! functional operator
        result_type operator()(const first_argument_type & lhs, const second_argument_type & rhs) const;
    };

    /*!
      \class XYCmp
      \brief comparison predicate for XY value (X -> Y order).
     */
    class XYCmp
        : public std::binary_function< Vector2D, Vector2D, bool > {
    public:
        /*!
          \brief functional operator.
          \param lhs left hand side argument.
          \param rhs right hand side argument.
          \return compared result.
         */
        result_type operator()(const first_argument_type & lhs, const second_argument_type & rhs) const;

    };

    /*!
      \class YXCmp
      \brief comparison predicatio for XY value (Y -> X order)
     */
    class YXCmp
        : public std::binary_function< Vector2D, Vector2D, bool > {
    public:
        /*!
          \brief functional operator.
          \param lhs left hand side argument.
          \param rhs right hand side argument.
          \return compared result.
         */
        result_type operator()(const first_argument_type & lhs, const second_argument_type & rhs) const;
    };


    //////////////////////////////////////////////
    // functor for region

    /*!
      \class IsWithin
      \brief template predicate for 2D region sign detection.
     */
    template < typename REGION >
    class IsWithin : public std::unary_function< Vector2D, bool > {
    private:
        const REGION M_region; //!< considered region.
    public:
        //! constructor
        explicit IsWithin(const REGION & region): M_region(region) {}

        //! functional operator
        result_type operator()(const argument_type & position) const {
            return M_region.contains(position);
        }
    };
};


} // end of namespace


////////////////////////////////////////////////////////
// comparison operators
/*!
  \brief check vectors are same or not.
  \param lhs left hand side parameter
  \param rhs right hand side parameter
  \return true if vectors are same.
*/
inline
bool
operator==(const rcsc::Vector2D & lhs, const rcsc::Vector2D & rhs) {
    return fabs(lhs.x - rhs.x) < 1e-3 && fabs(lhs.y - rhs.y) < 1e-3;
}

/*!
  \brief check vectors are different or not.
  \param lhs left hand side parameter
  \param rhs right hand side parameter
  \return true if vectors are not same.
*/
inline
bool
operator!=(const rcsc::Vector2D & lhs, const rcsc::Vector2D & rhs) {
    return !operator==(lhs, rhs);
}


////////////////////////////////////////////////////////
// arithmetic operators

/*!
  \brief operator add(T, T)
  \param lhs left hand side parameter
  \param rhs right hand side parameter
  \return new vector object
*/
inline
const
rcsc::Vector2D
operator+(const rcsc::Vector2D & lhs,
          const rcsc::Vector2D & rhs) {
    return rcsc::Vector2D(lhs) += rhs;
}

/*!
  \brief operator sub(T, T)
  \param lhs left hand side parameter
  \param rhs right hand side parameter
  \return new vector object
*/
inline
const
rcsc::Vector2D
operator-(const rcsc::Vector2D & lhs,
          const rcsc::Vector2D & rhs) {
    return rcsc::Vector2D(lhs) -= rhs;
}


/*!
  \brief operator mult(T, U)
  \param lhs left hand side parameter
  \param rhs right hand side parameter. double type
  \return new vector object
*/
inline
double
operator*(const rcsc::Vector2D & lhs,
          const rcsc::Vector2D & rhs) {
    return lhs.innerProduct(rhs);
}


/*!
  \brief operator mult(T, U)
  \param lhs left hand side parameter
  \param rhs right hand side parameter. double type
  \return new vector object
*/
inline
double
operator^(const rcsc::Vector2D & lhs,
          const rcsc::Vector2D & rhs) {
    return lhs.outerProduct(rhs);
}

/*!
  \brief operator mult(T, U)
  \param lhs left hand side parameter
  \param rhs right hand side parameter. double type
  \return new vector object
*/
inline
const
rcsc::Vector2D
operator*(const rcsc::Vector2D & lhs,
          const double & rhs) {
    return rcsc::Vector2D(lhs) *= rhs;
}


/*!
  \brief operator mult(T, U)
  \param lhs left hand side parameter
  \param rhs right hand side parameter. double type
  \return new vector object
*/
inline
const
rcsc::Vector2D
operator*(const double & rhs ,
          const rcsc::Vector2D & lhs) {
    return rcsc::Vector2D(lhs) *= rhs;
}

/*!
  \brief operator div(T, U)
  \param lhs left hand side parameter
  \param rhs right hand side parameter. double type
  \return new vector object
*/
inline
const
rcsc::Vector2D
operator/(const rcsc::Vector2D & lhs,
          const double & rhs) {
    return rcsc::Vector2D(lhs) /= rhs;
}

/*!
  \brief never used
 */
template < typename T >
bool
operator<(const rcsc::Vector2D & lhs,
          const T & rhs);

/*!
  \brief never used
 */
template < typename T >
bool
operator<=(const rcsc::Vector2D & lhs,
           const T & rhs);

/*!
  \brief never used
 */
template < typename T >
bool
operator>(const rcsc::Vector2D & lhs,
          const T & rhs);

/*!
  \brief never used
 */
template < typename T >
bool
operator>=(const rcsc::Vector2D & lhs,
           const T & rhs);

/*!
  \brief never used
 */
template < typename T >
bool
operator<(const T & lhs,
          const rcsc::Vector2D & rhs);


/*!
  \brief never used
 */
template < typename T >
bool
operator<=(const T & lhs,
           const rcsc::Vector2D & rhs);


/*!
  \brief never used
 */
template < typename T >
bool
operator>(const T & lhs,
          const rcsc::Vector2D & rhs);

/*!
  \brief never used
 */
template < typename T >
bool
operator>=(const T & lhs,
           const rcsc::Vector2D & rhs);

/*!
  \brief never used
 */
template < typename T >
bool
operator==(const T & lhs,
           const rcsc::Vector2D & rhs);

/*!
  \brief never used
 */
template < typename T >
bool
operator!=(const T & lhs,
           const rcsc::Vector2D & rhs);



////////////////////////////////////////////////////////

/*!
  \brief stream operator
  \param os reference to ostream
  \param v output value
  \return reference to ostream
*/
inline
std::ostream &
operator<<(std::ostream & os,
           const rcsc::Vector2D & v) {
    return v.print(os);
}


int sign(double d1);
double max(double d1, double d2);
double min(double d1, double d2);
namespace rcsc {
Vector2D intersect_ellipse_dir(Vector2D dir, Vector2D center, double a, double b, double e);
bool intersect_ellipse_line(Vector2D point1, Vector2D point2, Vector2D center, double _a, double _b, Vector2D * sol1, Vector2D * sol2);
}

#endif
