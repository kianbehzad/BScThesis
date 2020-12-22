// -*-c++-*-

/*!
  \file rect_2d.h
  \brief 2D rectangle region Header File.
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

#ifndef RCSC_GEOM_RECT2D_H
#define RCSC_GEOM_RECT2D_H

#include <pack_util/geom/size_2d.h>
#include <pack_util/geom/line_2d.h>
#include <pack_util/geom/vector_2d.h>
#include <pack_util/geom/circle_2d.h>

namespace rcsc {

class Ray2D;
class Segment2D;

/*!
  \class Rect2D
  \brief 2D rectangle regin class.

  The model and naming rules are depend on soccer simulator environment
          -34.0
            |
            |
-52.5 ------+------- 52.5
            |
            |
          34.0
*/
class Rect2D {
private:
    //! top left point
    Vector2D M_top_left;

    //! XY range
    Size2D M_size;

public:
    /*!
      \brief default constructor creates a zero area rectanble at (0,0)
     */
    Rect2D();

    /*!
      \brief constructor
      \param left_x left x
      \param top_y top y
      \param length length (x-range)
      \param width width (y-range)
     */
    Rect2D(const double & left_x,
           const double & top_y,
           const double & length,
           const double & width);
    /*!
      \brief constructor with variables
      \param top_left top left point
      \param length X range
      \param width Y range
     */
    Rect2D(const Vector2D & top_left,
           const double & length,
           const double & width);
    /*!
      \brief constructor with variables
      \param top_left top left point
      \param size XY range
     */
    Rect2D(const Vector2D & top_left,
           const Size2D & size);

    /*!
      \brief constructor with 2 points.
      \param top_left top left vertex
      \param bottom_right bottom right vertex

      Even if argument point has incorrect values,
      the assigned values are normalized automatically.
    */
    Rect2D(const Vector2D & top_left,
           const Vector2D & bottom_right);

    /*!
      \brief create rectangle with center point and size.
      \param center center point of new rectangle.
      \param length length(x-range) of new rectangle.
      \param width width(y-range) of new rectangle.
     */
    static
    Rect2D from_center(const Vector2D & center,
                       const double & length,
                       const double & width);

    /*!
      \brief create rectangle with center point and size.
      \param center_x x value of center point of new rectangle.
      \param center_y y value of center point of new rectangle.
      \param length length(x-range) of new rectangle.
      \param width width(y-range) of new rectangle.
     */
    static
    Rect2D from_center(const double & center_x,
                       const double & center_y,
                       const double & length,
                       const double & width);

    /*!
      \brief create rectangle with 2 corner points. just call one of constructor.
      \param top_left top left vertex
      \param bottom_right bottom right vertex
    */
    static
    Rect2D from_corners(const Vector2D & top_left,
                        const Vector2D & bottom_right);

    /*!
      \brief assign new values
      \param left_x left x
      \param top_y top y
      \param length X range
      \param width Y range
     */
    const
    Rect2D & assign(const double & left_x,
                    const double & top_y,
                    const double & length,
                    const double & width);

    /*!
      \brief assign new values
      \param top_left top left point
      \param length X range
      \param width Y range
      \return const referenct to itself
     */
    const
    Rect2D & assign(const Vector2D & top_left,
                    const double & length,
                    const double & width);

    /*!
      \brief assign new values
      \param top_left top left
      \param size XY range
      \return const referenct to itself
     */
    const
    Rect2D & assign(const Vector2D & top_left,
                    const Size2D & size);

    /*!
      \brief set a new top left corner point
      \param x new x coordinate
      \param y new y coordinate
      \return const referenct to itself
     */
    const
    Rect2D & setTopLeft(const double & x,
                        const double & y);

    /*!
      \brief set a new top left corner point
      \param point new coordinate
      \return const referenct to itself
     */
    const
    Rect2D & setTopLeft(const Vector2D & point);

    /*!
      \brief set a new center point. only top left corner is moved.
      size is not changed.
      \param point new center coordinate
      \return const referenct to itself
     */
    const
    Rect2D & setCenter(const Vector2D & point);

    /*!
      \brief set a new x-range
      \param length new range
      \return const referenct to itself
     */
    const
    Rect2D & setLength(const double & length);

    /*!
      \brief set a new y-range
      \param width new range
      \return const referenct to itself
     */
    const
    Rect2D & setWidth(const double & width);

    /*!
      \brief set a new size
      \param length new range
      \param width new range
      \return const referenct to itself
     */
    const
    Rect2D & setSize(const double & length,
                     const double & width);

    /*!
      \brief set a new size
      \param size new range
      \return const referenct to itself
     */
    const
    Rect2D & setSize(const Size2D & size);

    /*!
      \brief check if point is within this region.
      \param point considered point
      \return true or false
     */
    bool contains(const Vector2D & point) const;

    /*!
      \brief get the left x coordinate of this rectangle.
      \return x coordinate value
    */
    const
    double & left() const;

    /*!
      \brief get the right x coordinate of this rectangle.
      \return x coordinate value
    */
    double right() const;

    /*!
      \brief get the top y coordinate of this rectangle.
      \return y coordinate value
    */
    const
    double & top() const;

    /*!
      \brief get the bottom y coordinate of this rectangle.
      \return y coordinate value
    */
    double bottom() const;

    /*!
      \brief get minimum value of x coordinate of this rectangle
      \return x coordinate value (equivalent to left())
    */
    double minX() const;

    /*!
      \brief get maximum value of x coordinate of this rectangle
      \return x coordinate value (equivalent to right())
    */
    double maxX() const;

    /*!
      \brief get minimum value of y coordinate of this rectangle
      \return y coordinate value (equivalent to top())
    */
    double minY() const;

    /*!
      \brief get maximum value of y coordinate of this rectangle
      \return y coordinate value (equivalent to bottom())
    */
    double maxY() const;

    /*!
      \brief get the XY range of this rectangle
      \return size object
    */
    const
    Size2D & size() const;

    /*!
      \brief get center point
      \return coordinate value by vector object
     */
    Vector2D center() const;

    /*!
      \brief get the top-left corner point
      \return coordiante value by vector object
    */
    const
    Vector2D & topLeft() const;

    /*!
      \brief get the top-right corner point
      \return coordiante value by vector object
    */
    Vector2D topRight() const;

    /*!
      \brief get the bottom-left corner point
      \return coordiante value by vector object
    */
    Vector2D bottomLeft() const;

    /*!
      \brief get the bottom-right corner point
      \return coordiante value by vector object
    */
    Vector2D bottomRight() const;

    /*!
      \brief get the left edge line
      \return line object
    */
    Line2D leftEdge() const;

    /*!
      \brief get the right edge line
      \return line object
    */
    Line2D rightEdge() const;

    /*!
      \brief get the top edge line
      \return line object
    */
    Line2D topEdge() const;

    /*!
      \brief get the bottom edge line
      \return line object
    */
    Line2D bottomEdge() const;

    /*!
      \brief calculate intersection point with line.
      \param line considerd line.
      \param sol1 pointer to the 1st solution variable
      \param sol2 pointer to the 2nd solution variable
      \return number of intersection
    */
    int intersection(const Line2D & line,
                     Vector2D * sol1,
                     Vector2D * sol2) const;

    /*!
      \brief calculate intersection point with ray.
      \param ray considerd ray line.
      \param sol1 pointer to the 1st solution variable
      \param sol2 pointer to the 2nd solution variable
      \return number of intersection
    */
    int intersection(const Ray2D & ray,
                     Vector2D * sol1,
                     Vector2D * sol2) const;

    /*!
      \brief calculate intersection point with line segment.
      \param segment considerd line segment.
      \param sol1 pointer to the 1st solution variable
      \param sol2 pointer to the 2nd solution variable
      \return number of intersection
    */
    int intersection(const Segment2D & segment,
                     Vector2D * sol1,
                     Vector2D * sol2) const;

    int intersection(const Circle2D & circle,
                     Vector2D * sol1,
                     Vector2D * sol2,
                     Vector2D * sol3,
                     Vector2D * sol4) const;

    int rotateAndintersect(const Segment2D & segment, Vector2D center, float angle ,
                           Vector2D * sol1,
                           Vector2D * sol2) const;

    int rotateAndintersect(const Circle2D & circle, Vector2D center, float angle ,
                           Vector2D * sol1,
                           Vector2D * sol2,
                           Vector2D * sol3,
                           Vector2D * sol4) const;
};

}

#endif
