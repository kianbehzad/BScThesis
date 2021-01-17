//
// Created by Kian Behzad on 1/16/21.
//

#ifndef PACK_UTIL_DRAWER_H
#define PACK_UTIL_DRAWER_H

#include <string>

#include "pack_util/geom/vector_2d.h"

#include "pack_msgs/msg/shapes.hpp"
#include "pack_msgs/msg/vector2_d.hpp"
#include "pack_msgs/msg/shape_rect.hpp"
#include "pack_msgs/msg/shape_triangle.hpp"
#include "pack_msgs/msg/shape_line.hpp"

class Drawer
{
public:
    explicit Drawer(std::string publisher_name = "default");
    ~Drawer();

    void erase_draws();
    pack_msgs::msg::Shapes get_draws();

    void choose_pen(const std::string& _color, const bool& _fill_the_shape);

    void draw_point(const double& x, const double& y);
    void draw_point(const rcsc::Vector2D& vec);
    void draw_point(const pack_msgs::msg::Vector2D& vec);
    void draw_rect(const double& left_top_x, const double& left_top_y, const double& width, const double& height);
    void draw_rect(const rcsc::Vector2D& left_top, const double& width, const double& height);
    void draw_rect(const pack_msgs::msg::ShapeRect& rect);
    void draw_circle(const double& center_x, const double& center_y, const double& r);
    void draw_circle(const rcsc::Vector2D& center, const double& r);
    void draw_ellipse(const double& left_top_x, const double& left_top_y, const double& width, const double& height);
    void draw_ellipse(const rcsc::Vector2D& left_top, const double& width, const double& height);
    void draw_ellipse(const pack_msgs::msg::ShapeRect& rect);
    void draw_triangle(const double& x1, const double& y1, const double& x2, const double& y2, const double& x3, const double& y3);
    void draw_triangle(const rcsc::Vector2D& p1, const rcsc::Vector2D& p2, const rcsc::Vector2D& p3);
    void draw_triangle(const pack_msgs::msg::ShapeTriangle& tri);
    void draw_line(const double& x1, const double& y1, const double& x2, const double& y2);
    void draw_line(const rcsc::Vector2D& p1, const rcsc::Vector2D& p2);
    void draw_line(const pack_msgs::msg::ShapeLine& line);

private:
    pack_msgs::msg::Shapes* shapes;
    std::string publisher_name;
    std::string color;
    bool fill_the_shape;

};


#endif //PACK_UTIL_DRAWER_H
