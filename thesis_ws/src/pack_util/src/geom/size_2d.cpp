//
// Created by kian behzad on 4/5/20.
//

#include "pack_util/geom/size_2d.h"

namespace rcsc{

    Size2D::Size2D() : M_length(0.0), M_width(0.0) {}


    Size2D::Size2D(const double & length, const double & width) : M_length(std::fabs(length)), M_width(std::fabs(width)) {}


    const Size2D & Size2D::assign(const double & length, const double & width) {
        M_length = std::fabs(length);
        M_width = std::fabs(width);
        return *this;
    }


    const Size2D & Size2D::setLength(const double & length) {
        M_length = std::fabs(length);
        return *this;
    }


    const Size2D & Size2D::setWidth(const double & width) {
        M_width = std::fabs(width);
        return *this;
    }


    const double & Size2D::length() const {
        return M_length;
    }


    const double & Size2D::width() const {
        return M_width;
    }


    double Size2D::diagonal() const {
        return std::sqrt(length() * length()
                         + width() * width());
    }


    std::ostream & Size2D::print(std::ostream & os) const {
        os << "(" << length() << ", " << width() << ")";
        return os;
    }


} // end of namespace
