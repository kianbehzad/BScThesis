// -*-c++-*-

/*!
  \file circle_2d.h
  \brief 2D circle region Header File.
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

#ifndef RCSC_GEOM_CIRCLE2D_H
#define RCSC_GEOM_CIRCLE2D_H

#include <iostream>

#include <pack_util/geom/vector_2d.h>

#include "pack_util/geom/segment_2d.h"

namespace rcsc {

class Line2D;
class Ray2D;

/*!
  \class Circle2D
  \brief 2d circle class
 */
class Circle2D {
private:

    //! center point
    Vector2D M_center;

    //! radius of this circle
    double M_radius;

    static const double EPSILOON;


public:
    /*!
      \brief create a zero area circle at (0,0)
     */
    Circle2D();


    /*!
      \brief constructor with all values
      \param c center point
      \param r radius value
     */
    Circle2D(const Vector2D & c, const double & r);

    /*!
      \brief assign new value.
      \param c center point
      \param r radius value
      \return const reference to this
     */
    const
    Circle2D & assign(const Vector2D & c,
                      const double & r);
    /*!
      \brief check if point is within this region
      \param point considered point
      \return true if point is contained by this circle
     */
    bool contains(const Vector2D & point) const;

    /*!
      \brief get the center point
      \return center point coordinate value
     */
    const
    Vector2D & center() const;

    /*!
      \brief get the radius value
      \return radius value
     */
    const
    double & radius() const;

    /*!
      \brief caluclate the intersection with straight line
      \param line considerd line
      \param sol1 pointer to the 1st solution variable
      \param sol2 pointer to the 2nd solution variable
      \return the number of solution
     */
    int intersection(const Line2D & line,
                     Vector2D * sol1,
                     Vector2D * sol2) const;

    /*!
      \brief calculate the intersection with ray line
      \param ray considerd ray
      \param sol1 pointer to the 1st solution variable
      \param sol2 pointer to the 2nd solution variable
      \return the number of solution
     */
    int intersection(const Ray2D & ray,
                     Vector2D * sol1,
                     Vector2D * sol2) const;

    /*!
      \brief calculate the intersection with another circle
      \param circle considerd circle
      \param sol1 pointer to the 1st solution variable
      \param sol2 pointer to the 2nd solution variable
      \return the number of solution
     */
    int intersection(const Circle2D & circle,
                     Vector2D * sol1,
                     Vector2D * sol2) const;
    /*!
      \brief calculate the intersection with segment
      \param seg considerd segment
      \param sol1 pointer to the 1st solution variable
      \param sol2 pointer to the 2nd solution variable
      \return the number of solution
     */


    int intersection(const Segment2D & seg,
                     Vector2D * sol1,
                     Vector2D * sol2) const;


    /*!
      \brief calculate the intersection with segment
      \param seg considerd segment
      \return the number of solution
     */


    int intersection(const Segment2D & seg) const;

    int tangent(Vector2D p, Vector2D * sol1, Vector2D * sol2);

    // static utility

    /*!
      \brief get the circumcircle from triangle vertexs
      \param a triangle's 1st vertex
      \param b triangle's 2nd vertex
      \param c triangle's 3rd vertex
      \return coordinates of circumcenter
    */
    static
    Circle2D circumcircle(const Vector2D & a,
                          const Vector2D & b,
                          const Vector2D & c);
};

}

#endif
