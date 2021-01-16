//
// Created by Kian Behzad on 1/16/21.
//

#include "pack_util/core/drawer.h"

#include <utility>

Drawer::Drawer(std::string _publisher_name) : publisher_name{_publisher_name}
{
    shapes = new pack_msgs::msg::Shapes;
    shapes->publisher_name = publisher_name;
}

Drawer::~Drawer()
{
   erase_draws();
}

pack_msgs::msg::Shapes Drawer::get_draws()
{
    pack_msgs::msg::Shapes tmp;
    if(shapes != nullptr) tmp = *shapes;
    erase_draws();
    shapes = new pack_msgs::msg::Shapes;
    shapes->publisher_name = publisher_name;
    return tmp;
}

void Drawer::erase_draws()
{
    if(shapes != nullptr)
    {
        shapes->points.clear();
        shapes->rects.clear();
        shapes->circles.clear();
        shapes->triangles.clear();
        shapes->lines.clear();
        delete shapes; shapes = nullptr;
    }
}

void Drawer::choose_pen(const std::string& _color, const bool& _fill_the_shape)
{
    color = _color;
    fill_the_shape = _fill_the_shape;
}

void Drawer::draw_point(const double& x, const double& y)
{
    pack_msgs::msg::Vector2D tmp;
    tmp.x = x;  tmp.y = y;
    shapes->points.push_back(tmp);
}
void Drawer::draw_point(const rcsc::Vector2D& vec)
{
    pack_msgs::msg::Vector2D tmp;
    tmp.x = vec.x;  tmp.y = vec.y;
    shapes->points.push_back(tmp);
}
void Drawer::draw_point(const pack_msgs::msg::Vector2D& vec)
{
    pack_msgs::msg::Vector2D tmp = vec;
    shapes->points.push_back(tmp);
}

void Drawer::draw_rect(const double& left_top_x, const double& left_top_y, const double& width, const double& height)
{
    pack_msgs::msg::ShapeRect tmp;
    tmp.x = left_top_x; tmp.y = left_top_y;
    tmp.width = width;  tmp.height = height;
    tmp.fill = fill_the_shape;  tmp.color = color;
    shapes->rects.push_back(tmp);
}

void Drawer::draw_rect(const pack_msgs::msg::ShapeRect& rect)
{
    pack_msgs::msg::ShapeRect tmp = rect;
    shapes->rects.push_back(tmp);
}

void Drawer::draw_circle(const double& center_x, const double& center_y, const double& r)
{
    pack_msgs::msg::ShapeRect tmp;
    tmp.x = center_x-r; tmp.y = center_y+r;
    tmp.width = 2*r;    tmp.height = 2*r;
    tmp.fill = fill_the_shape;  tmp.color = color;
    shapes->circles.push_back(tmp);
}

void Drawer::draw_ellipse(const double& left_top_x, const double& left_top_y, const double& width, const double& height)
{
    pack_msgs::msg::ShapeRect tmp;
    tmp.x = left_top_x; tmp.y = left_top_y;
    tmp.width = width;  tmp.height = height;
    tmp.fill = fill_the_shape;  tmp.color = color;
    shapes->circles.push_back(tmp);
}
void Drawer::draw_ellipse(const pack_msgs::msg::ShapeRect& rect)
{
    pack_msgs::msg::ShapeRect tmp = rect;
    shapes->circles.push_back(tmp);
}

void Drawer::draw_triangle(const double& x1, const double& y1, const double& x2, const double& y2, const double& x3, const double& y3)
{
    pack_msgs::msg::ShapeTriangle tmp;
    tmp.x1 = x1;    tmp.y1 = y1;
    tmp.x2 = x2;    tmp.y1 = y2;
    tmp.x3 = x3;    tmp.y1 = y3;
    tmp.fill = fill_the_shape;  tmp.color = color;
    shapes->triangles.push_back(tmp);
}

void Drawer::draw_triangle(const pack_msgs::msg::ShapeTriangle& tri)
{
    pack_msgs::msg::ShapeTriangle tmp = tri;
    shapes->triangles.push_back(tmp);
}

void Drawer::draw_line(const double& x1, const double& y1, const double& x2, const double& y2)
{
    pack_msgs::msg::ShapeLine tmp;
    tmp.x1 = x1;    tmp.y1 = y1;
    tmp.x2 = x2;    tmp.y2 = y2;
    tmp.color = color;
    shapes->lines.push_back(tmp);
}
void Drawer::draw_line(const pack_msgs::msg::ShapeLine& line)
{
    pack_msgs::msg::ShapeLine tmp = line;
    shapes->lines.push_back(tmp);
}



