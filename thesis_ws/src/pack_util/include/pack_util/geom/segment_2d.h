// -*-c++-*-

/*!
  \file segment_2d.h
  \brief 2D segment line Header File.
*/

/*
 *Copyright:

 Copyright (C) Hidehisa Akiyama, Hiroki Shimora

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

#ifndef RCSC_GEOM_SEGMENT2D_H
#define RCSC_GEOM_SEGMENT2D_H

#include <pack_util/geom/line_2d.h>
#include <pack_util/geom/vector_2d.h>

#include <cmath>

namespace rcsc {

/*!
  \class Segment2D
  \brief 2d segment line class
*/
class Segment2D {
private:

    Vector2D M_a; //!< first point
    Vector2D M_b; //!< second point



    bool checkIntersectsOnLine(const Vector2D & p) const;

public:

    //! not used
    Segment2D();

    /*!
      \brief construct from 2 points
      \param a 1st point of segment edge
      \param b 2nd point of segment edge
     */
    Segment2D(const Vector2D & a,
              const Vector2D & b);

    /*!
      \brief construct directly using raw coordinate values
      \param ax 1st point x value of segment edge
      \param ay 1st point x value of segment edge
      \param bx 1st point y value of segment edge
      \param by 1st point y value of segment edge
     */
    Segment2D(const double & ax,
              const double & ay,
              const double & bx,
              const double & by);
    /*!
      \brief construct using origin, direction and length
      \param origin origin point
      \param length length of line segment
      \param dir line direction from origin point
     */
    Segment2D( const Vector2D & a,
               const double & length,
               const AngleDeg & dir );

    /*!
      \brief construct from 2 points
      \param a first point
      \param b second point
      \return const reference to itself
    */
    const
    Segment2D & assign(const Vector2D & a,
                       const Vector2D & b);

    /*!
      \brief construct directly using raw coordinate values
      \param ax 1st point x value of segment edge
      \param ay 1st point x value of segment edge
      \param bx 1st point y value of segment edge
      \param by 1st point y value of segment edge
    */
    const
    Segment2D & assign(const double & ax,
                       const double & ay,
                       const double & bx,
                       const double & by);
    /*!
      \brief construct using origin, direction and length
      \param origin origin point
      \param length length of line segment
      \param dir line direction from origin point
      \return const reference to this object
    */
    const
    Segment2D & assign( const Vector2D & a,
                        const double & length,
                        const AngleDeg & dir );
    /*!
      \brief swap segment edge point
      \return const reference to itself
    */
    const
    Segment2D & swap();

    /*!
      \brief get 1st point of segment edge
      \return vector object
    */
    const
    Vector2D & a() const;

    /*!
      \brief get 2nd point of segment edge
      \return vector object
    */
    const
    Vector2D & b() const;

    const
    Vector2D & origin() const;

    /*!
      \brief get 2nd point of segment edge
      \return const reference to the vector object
    */
    const
    Vector2D & terminal() const;


    /*!
      \brief get line generated from segment
      \return new line object
    */
    Line2D line() const;

    /*!
      \brief get the length of this segment
      \return distance value
     */
    double length() const;

    /*!
      \brief make perpendicular bisector line from segment points
      \return line object
     */
    Line2D perpendicularBisector() const;
    /*!
      \brief check if the point is within the rectangle defined by this
      segment as a diagonal line.
      \return true if rectangle contains p
     */
    bool contains(const Vector2D & p) const;

    /*!
      \brief check & get the intersection point with other line segment
      \param other considered line segment
      \return intersection point. if it does not exist,
      the invalidated value vector is returned.
    */
    Vector2D intersection(const Segment2D & other) const;

    /*!
      \brief check & get the intersection point with other line
      \param other considered line
      \return intersection point. if it does not exist,
      the invalidated value vector is returned.
    */
    Vector2D intersection(const Line2D & other) const;

    /*!
      \brief check if segments cross each other or not.
      \param other segment for cross checking
      \return true if this segment crosses, otherwise returns false.
    */
    bool existIntersection(const Segment2D & other) const;

    /*!
      \brief check if segments intersect each other on non terminal point.
      \param other segment for cross checking
      \return true if this segments intersect and intersection point is not a
      terminal point of segment.
      false if segments not intersect or intersect on terminal point of segment.
    */
    bool existIntersectionExceptEndpoint(const Segment2D & other) const;

    /*!
      \brief get a point on segment where distance of point is minimal.
      \param p point
      \return nearest point on segment. if multiple nearest points found.
       returns one of them.
    */
    Vector2D nearestPoint(const Vector2D & p) const;

    /*!
      \brief get minimum distance between this segment and point
      \param p point
      \return minimum distance between this segment and point
    */
    double dist(const Vector2D & p) const;

    /*!
      \brief get minimum distance between 2 segments
      \param seg segment
      \return minimum distance between 2 segments
    */
    double dist(const Segment2D & seg) const;

    /*!
      \brief get maximum distance between this segment and point
      \param p point
      \return maximum distance between this segment and point
    */
    double farthestDist(const Vector2D & p) const;

    /*
      \brief check point is on segment or not
      \p point to check
      \p return true if point is on this segment
    */
    bool onSegment(const Vector2D & p) const;

    Vector2D projection(const Vector2D & p) const;
    bool onSegmentWeakly(const Vector2D & p) const;
};

}

#endif
