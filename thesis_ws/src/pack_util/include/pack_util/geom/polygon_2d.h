// -*-c++-*-

/*!
  \file polygon_2d.h
  \brief 2D polygon region Header File.
*/

/*
 *Copyright:

 Copyright (C) Hiroki Shimora

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

#ifndef RCSC_GEOM_POLYGON2D_H
#define RCSC_GEOM_POLYGON2D_H

#include <vector>
#include <iostream>

namespace rcsc {

class Vector2D;
class Rect2D;

/*!
  \class Polygon2D
  \brief 2D polygon region class
 */
class Polygon2D {
private:
    std::vector< Vector2D > M_vertex;

public:
    /*!
      \brief create empty polygon
    */
    Polygon2D();

    /*!
      \brief create polygon with points
      \param v array of points
    */
    Polygon2D( const std::vector< Vector2D > & v );

    /*!
      \brief set polygon to empty polygon
      \return const reference to itself
    */
    const Polygon2D & assign();

    /*!
      \brief set polygon with points
      \param v array of points
      \return const reference to itself
    */
    const Polygon2D & assign( const std::vector< Vector2D > & v );

    /*!
      \brief append point to polygon
      \param p new point
    */
    void addVertex( const Vector2D & p );

    /*!
      \brief get list of point of this polygon
      \return const reference to point list
    */
    const std::vector< Vector2D > & vertex() const;

    /*!
      \brief get list of point of this polygon
      \return reference to point list
    */
    std::vector< Vector2D > & vertex();

    /*!
      \brief check point is in this polygon or not
      \param p point for checking
      \param allow_on_segment when point is on outline,
      if this parameter is set to true, returns true
      \return true if point is in this polygon
    */
    bool contains( const Vector2D & p,
                   bool allow_on_segment = true ) const;

    /*!
      \brief get bounding box of this polygon
      \return bounding box of this polygon
    */
    Rect2D getBoundingBox() const;

    /*!
      \brief get centor of bounding box of this polygon
      \return centor of bounding box of this polygon
    */
    Vector2D xyCenter() const;

    /*!
      \brief get minimum distance between this polygon and point
      \param p point
      \param check_as_plane if this parameter is set to true, handle this
      polygon as a plane polygon,
      otherwise handle this polygon as a polyline polygon.
      when point is inside of this polygon, distance between plane polygon
      and point is 0,
      distance between polyline polygon and point is minimum distance
      between each segments of this polygon.
      \return minimum distance between this polygon and point
    */
    double dist( const Vector2D & p,
                 bool check_as_plane = true ) const;

    /*!
      \brief get area of this polygon
      \return value of area with sign.
    */
    double area() const;

    /*!
      \brief get area with sign
      \return value of area with sign.
      If vertexes are placed counterclockwise order, returns positive number.
      If vertexes are placed clockwise order, returns negative number.
      Otherwise, returns 0.
    */
    double signedArea2() const;

    /*!
      \brief check vertexes of this polygon is placed counterclockwise ot not
      \return true if counterclockwise
    */
    bool isCounterclockwise() const;

    /*!
      \brief check vertexes of this polygon is placed clockwise ot not
      \return true if clockwise
    */
    bool isClockwise() const;

    /*!
      \brief get a polygon clipped by a rectangle
      \param r rectangle for clipping
      \return a polygon. if polygon is separated by edges of rectangle,
      each separated polygon is connected to one polygon.
    */
    Polygon2D getScissoredConnectedPolygon( const Rect2D & r ) const;
};

} // end of namespace

#endif
