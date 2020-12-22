// -*-c++-*-

/*!
  \file line_2d.h
  \brief 2D straight line Header File.
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

#ifndef RCSC_GEOM_LINE2D_H
#define RCSC_GEOM_LINE2D_H

#include <pack_util/geom/vector_2d.h>

#include <cmath>
#include <iostream>
#include <limits>

namespace rcsc {

/*!
  \class Line2D
  \brief 2d straight line class

  Line Fomula: aX + bY + c = 0
*/
class Line2D {
public:

    static const double EPSILOON; //! epsilon value
    static const double ERROR_VALUE; //! epsilon value

private:

    double M_a; //!< line fomula A, coefficient for x
    double M_b; //!< line fomula B, coefficient for y
    double M_c; //!< line fomula constant C

    // never used
    Line2D();

public:
    /*!
      \brief construct directly
      \param a Line formula A, coefficient for x
      \param b Line formula B, coefficient for y
      \param c constant C
     */
    Line2D(const double & a, const double & b, const double & c);
    /*!
      \brief construct from 2 points
      \param p1 first point
      \param p2 second point
    */
    Line2D(const Vector2D & p1, const Vector2D & p2);

    /*!
      \brief construct from origin point + direction
      \param origin origin point
      \param linedir direction from origin point
    */
    Line2D(const Vector2D & origin, const AngleDeg & linedir);

    /*!
       \brief construct from 2 points
       \param p1 first point
       \param p2 second point
       \return const reference to itself
     */
    const Line2D & assign(const Vector2D & p1, const Vector2D & p2);

    /*!
       \brief construct from origin point + direction
       \param origin origin point
       \param linedir direction from origin point
       \return const reference to itself
     */
    const Line2D & assign(const Vector2D & origin, const AngleDeg & linedir);
    /*!
      \brief accessor
      \return coefficient 'A' of line formula
    */
    const double & a() const;

    /*!
      \brief accessor
      \return coefficient 'A' of line formula
    */
    const double & getA() const;

    /*!
      \brief accessor
      \return coefficient 'B'  of line formula
    */
    const double & b() const;

    /*!
      \brief accessor
      \return coefficient 'A' of line formula
    */
    /*!
      \brief accessor
      \return coefficient 'B'  of line formula
    */
    const double & getB() const;

    /*!
      \brief accessor
      \return coefficient 'C'  of line formula
    */
    const double & c() const;

    /*!
      \brief accessor
      \return coefficient 'C'  of line formula
    */
    const double & getC() const;

    /*!
      \brief get X-coordinate correspond to 'y'
      \param y considered Y
      \return X coordinate
    */
    double getX(const double & y) const;

    /*!
      \brief get Y-coordinate correspond to 'x'
      \param x considered X
      \return Y coordinate
    */
    double getY(const double & x) const;

    /*!
      \brief calculate distance from point to this line
      \param p considrered point
      \return distance value
    */
    double dist(const Vector2D & p) const;
    /*!
      \brief get squared distance from this line to point
      \param p considrered point
      \return squared distance value
    */
    double dist2(const Vector2D & p) const;

    /*!
      \brief check if the slope of this line is same to the slope of 'line'
      \param line considered line
      \retval true almost same
      \retval false not same
    */
    bool isParallel(const Line2D & line) const;

    /*!
      \brief get the intersection point with 'line'
      \param line considered line
      \return intersection point. if it does not exist,
      the invaidated value vector is returned.
    */
    Vector2D intersection(const Line2D & line) const;

    /*!
      \brief calc perpendicular line (SUI-SEN)
      \param p the point that perpendicular line pass through
      \return perpendicular line
     */
    Line2D perpendicular(const Vector2D & p) const;

    /*!
      \brief calc projection point from p
      \param p base point
      \return projection point
     */
    Vector2D projection(const Vector2D & p) const;

    // static utility

    /*!
      \brief get the intersection point of 2 lines
      \param line1 the first line
      \param line2 the second line
      \return the intersection point.
      if no intersection, invalid vector is returned.
     */
    static Vector2D intersection(const Line2D & line1, const Line2D & line2);

    /*!
      \brief make angle bisector line from two angles
      \param origin origin point that is passed through by result line
      \param left left angle
      \param right right angle
      \return line object
     */
    static Line2D angle_bisector(const Vector2D & origin, const AngleDeg & left, const AngleDeg & right);

    /*!
      \brief make perpendicular bisector line from twt points
      \param p1 1st point
      \param p2 2nd point
      \return line object
     */
    static Line2D perpendicular_bisector(const Vector2D & p1, const Vector2D & p2);

};

}

#endif
