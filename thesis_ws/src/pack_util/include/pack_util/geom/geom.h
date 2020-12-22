// -*-c++-*-

/*!
  \file geom.h
  \brief geometry library Header File
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

#ifndef RCSC_GEOM_H
#define RCSC_GEOM_H

#include <pack_util/geom/angle_deg.h>
#include <pack_util/geom/circle_2d.h>
#include <pack_util/geom/line_2d.h>
#include <pack_util/geom/matrix_2d.h>
#include <pack_util/geom/polygon_2d.h>
#include <pack_util/geom/ray_2d.h>
#include <pack_util/geom/rect_2d.h>
#include <pack_util/geom/sector_2d.h>
#include <pack_util/geom/segment_2d.h>
#include <pack_util/geom/size_2d.h>
#include <pack_util/geom/triangle_2d.h>
#include <pack_util/geom/vector_2d.h>

#define EPSILON 0.0001
#define _INF        1.0e13

using namespace rcsc;

#endif
