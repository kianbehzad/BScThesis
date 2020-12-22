// -*-c++-*-

/*!
  \file rect_2d.cpp
  \brief 2D rectangle region Source File.
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

#include "pack_util/geom/rect_2d.h"
#include "pack_util/geom/ray_2d.h"

namespace rcsc {


    Rect2D::Rect2D() : M_top_left(0.0, 0.0), M_size(0.0, 0.0) {}


    Rect2D::Rect2D(const double & left_x, const double & top_y, const double & length, const double & width) : M_top_left(left_x, top_y), M_size(length, width) {}


    Rect2D::Rect2D(const Vector2D & top_left, const double & length, const double & width) : M_top_left(top_left), M_size(length, width) {}


    Rect2D::Rect2D(const Vector2D & top_left, const Size2D & size) : M_top_left(top_left), M_size(size) {}


    Rect2D::Rect2D(const Vector2D & top_left, const Vector2D & bottom_right) : M_top_left(top_left), M_size(bottom_right.x - top_left.x, top_left.y - bottom_right.y) {
        if (bottom_right.x - top_left.x < 0.0) {
            M_top_left.x = bottom_right.x;
        }
        if (bottom_right.y - top_left.y > 0.0) {
            M_top_left.y = bottom_right.y;
        }
    }


    Rect2D Rect2D::from_center(const Vector2D & center, const double & length, const double & width) {
        return Rect2D(center.x - length * 0.5,
                      center.y + width * 0.5,
                      length,
                      width);
    }


    Rect2D Rect2D::from_center(const double & center_x, const double & center_y, const double & length, const double & width) {
        return Rect2D(center_x - length * 0.5,
                      center_y + width * 0.5,
                      length,
                      width);
    }


    Rect2D Rect2D::from_corners(const Vector2D & top_left, const Vector2D & bottom_right) {
        return Rect2D(top_left, bottom_right);
    }


    const Rect2D & Rect2D::assign(const double & left_x, const double & top_y, const double & length, const double & width) {
        M_top_left.assign(left_x, top_y);
        M_size.assign(length, width);
        return *this;
    }


    const Rect2D & Rect2D::assign(const Vector2D & top_left, const double & length, const double & width) {
        M_top_left = top_left;
        M_size.assign(length, width);
        return *this;
    }


    const Rect2D & Rect2D::assign(const Vector2D & top_left, const Size2D & size) {
        M_top_left = top_left;
        M_size = size;
        return *this;
    }


    const Rect2D & Rect2D::setTopLeft(const double & x, const double & y) {
        M_top_left.assign(x, y);
        return *this;
    }


    const Rect2D & Rect2D::setTopLeft(const Vector2D & point) {
        M_top_left = point;
        return *this;
    }


    const Rect2D & Rect2D::setCenter(const Vector2D & point) {
        M_top_left.assign(point.x - M_size.length() * 0.5,
                          point.y - M_size.width() * 0.5);
        return *this;
    }


    const Rect2D & Rect2D::setLength(const double & length) {
        M_size.setLength(length);
        return *this;
    }


    const Rect2D & Rect2D::setWidth(const double & width) {
        M_size.setWidth(width);
        return *this;
    }


    const Rect2D & Rect2D::setSize(const double & length,
                     const double & width) {
        M_size.assign(length, width);
        return *this;
    }


    const Rect2D & Rect2D::setSize(const Size2D & size) {
        M_size = size;
        return *this;
    }


    bool Rect2D::contains(const Vector2D & point) const {
        return (left() <= point.x
                && point.x <= right()
                && top() >= point.y
                && point.y >= bottom());
    }


    const double & Rect2D::left() const {
        return M_top_left.x;
    }


    double Rect2D::right() const {
        return left() + size().length();
    }


    const double & Rect2D::top() const {
        return M_top_left.y;
    }


    double Rect2D::bottom() const {
        return top() - size().width();
    }


    double Rect2D::minX() const {
        return left();
    }


    double Rect2D::maxX() const {
        return right();
    }


    double Rect2D::minY() const {
        return bottom();
    }


    double Rect2D::maxY() const {
        return top();
    }


    const Size2D & Rect2D::size() const {
        return M_size;
    }


    Vector2D Rect2D::center() const {
        return Vector2D((left() + right()) * 0.5,
                        (top() + bottom()) * 0.5);
    }


    const Vector2D & Rect2D::topLeft() const {
        return M_top_left;
    }


    Vector2D Rect2D::topRight() const {
        return Vector2D(right(), top());
    }


    Vector2D Rect2D::bottomLeft() const {
        return Vector2D(left(), bottom());
    }


    Vector2D Rect2D::bottomRight() const {
        return Vector2D(right(), bottom());
    }


    Line2D Rect2D::leftEdge() const {
        return Line2D(topLeft(), bottomLeft());
    }


    Line2D Rect2D::rightEdge() const {
        return Line2D(topRight(), bottomRight());
    }


    Line2D Rect2D::topEdge() const {
        return Line2D(topLeft(), topRight());
    }


    Line2D Rect2D::bottomEdge() const {
        return Line2D(bottomLeft(), bottomRight());
    }


    int Rect2D::intersection(const Line2D & line, Vector2D * sol1, Vector2D * sol2) const {
        int n_sol = 0;
        Vector2D tsol[2];

        const double left_x = left();
        const double right_x = right();
        const double top_y = top();
        const double bottom_y = bottom();

        if (n_sol < 2
            && (tsol[n_sol] = leftEdge().intersection(line)).valid()
            && top_y >= tsol[n_sol].y && tsol[n_sol].y >= bottom_y) {
            ++n_sol;
        }

        if (n_sol < 2
            && (tsol[n_sol] = rightEdge().intersection(line)).valid()
            && top_y >= tsol[n_sol].y && tsol[n_sol].y >= bottom_y) {
            ++n_sol;
        }

        if (n_sol < 2
            && (tsol[n_sol] = topEdge().intersection(line)).valid()
            && left_x <= tsol[n_sol].x && tsol[n_sol].x <= right_x) {
            ++n_sol;
        }

        if (n_sol < 2
            && (tsol[n_sol] = bottomEdge().intersection(line)).valid()
            && left_x <= tsol[n_sol].x && tsol[n_sol].x <= right_x) {
            ++n_sol;
        }

        if (n_sol > 0
            && sol1) {
            *sol1 = tsol[0];
        }

        if (n_sol > 1
            && sol2) {
            *sol2 = tsol[1];
        }

        return n_sol;
    }


    int Rect2D::intersection(const Ray2D & ray, Vector2D * sol1, Vector2D * sol2) const {
        Vector2D tsol1, tsol2;
        int n_sol = intersection(ray.line(), &tsol1, &tsol2);

        if (n_sol > 1
            && ! ray.inRightDir(tsol2, 1.0)) {
            --n_sol;
        }

        if (n_sol > 0
            && ! ray.inRightDir(tsol1, 1.0)) {
            tsol1 = tsol2;
            --n_sol;
        }

        if (n_sol > 0
            && sol1) {
            *sol1 = tsol1;
        }

        if (n_sol > 1
            && sol2) {
            *sol2 = tsol2;
        }

        return n_sol;
    }


    int Rect2D::intersection(const Segment2D & segment, Vector2D * sol1, Vector2D * sol2) const {
        Vector2D tsol1, tsol2;
        int n_sol = intersection(segment.line(), &tsol1, &tsol2);

        if (n_sol > 1
            && ! segment.contains(tsol2)) {
            --n_sol;
        }

        if (n_sol > 0
            && ! segment.contains(tsol1)) {
            tsol1 = tsol2;
            --n_sol;
        }

        if (n_sol > 0
            && sol1) {
            *sol1 = tsol1;
        }

        if (n_sol > 1
            && sol2) {
            *sol2 = tsol2;
        }

        return n_sol;
    }


    int Rect2D::intersection(const Circle2D & circle, Vector2D * sol1, Vector2D * sol2, Vector2D * sol3, Vector2D * sol4) const {
        int ni = 0;
        Vector2D sols[4] = {Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE),
                            Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE),
                            Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE),
                            Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE)
        };

        Segment2D segs[4];
        segs[0] = Segment2D(topLeft(), topRight());
        segs[1] = Segment2D(bottomLeft(), bottomRight());
        segs[2] = Segment2D(bottomLeft(), topLeft());
        segs[3] = Segment2D(bottomRight(), topRight());
        for (int i = 0; i < 4 && ni < 4; i++) {
            ni += circle.intersection(segs[i], &sols[ni], &sols[ni + 1]);
        }
        *sol1 = sols[0];
        *sol2 = sols[1];
        *sol3 = sols[2];
        *sol4 = sols[3];
        return ni;

    }


    int Rect2D::rotateAndintersect(const Segment2D & segment, Vector2D center, float angle , Vector2D * sol1, Vector2D * sol2) const {
        Vector2D a = segment.a() - center;
        Vector2D b = segment.b() - center;
        a.rotate(-angle);
        b.rotate(-angle);
        a += center;
        b += center;
        int res = intersection(Segment2D(a, b), sol1, sol2);
        sol1->rotate(angle);
        sol2->rotate(angle);

        return res;
    }


    int Rect2D::rotateAndintersect(const Circle2D & circle, Vector2D center, float angle , Vector2D * sol1, Vector2D * sol2, Vector2D * sol3, Vector2D * sol4) const {

        Vector2D cirCenter = circle.center() - center;
        cirCenter.rotate(-angle);
        cirCenter += center;
        int res = intersection(Circle2D(cirCenter, circle.radius()) , sol1, sol2, sol3, sol4);

        sol1->rotate(angle);
        sol2->rotate(angle);
        sol3->rotate(angle);
        sol4->rotate(angle);

        return res;
    }



} // end of namespace
