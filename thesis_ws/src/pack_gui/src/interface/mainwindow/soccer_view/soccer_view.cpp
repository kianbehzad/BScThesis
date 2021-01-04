//
// Created by Kian Behzad on 1/3/21.
//

#include "pack_gui/interface/mainwindow/soccer_view/soccer_view.h"

SoccerView::SoccerView(QWidget *parent) : QOpenGLWidget(parent)
{
    // redraw
    connect(this, SIGNAL(postRedraw()), this, SLOT(redraw()));
    scale_ratio = 1;
    blue_team_color.setNamedColor("cyan");
    yellow_team_color.setNamedColor("yellow");

    // create dynamic-reconfigure node
    node = rclcpp::Node::make_shared("soccer_view_node");

    // worldmodel callback
    worldmodel_subscription = node->create_subscription<pack_msgs::msg::WorldModel>("/world_model", 10, std::bind(&SoccerView::worldmodel_callback, this, std::placeholders::_1));
    // geometry callback
    geometry_subscription = node->create_subscription<pack_msgs::msg::SSLVisionGeometry>("/vision_geom", 10, std::bind(&SoccerView::geometry_callback, this, std::placeholders::_1));

    // run node in a different thread
    NodeInThread node_in_thread{node};
    std::thread _thread{node_in_thread};
    _thread.detach();

}

SoccerView::~SoccerView()
{

}

void SoccerView::initializeGL()
{
    glClearColor(0.0, 0.5686, 0.0980, 1.0);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

}

void SoccerView::paintGL ()
{
    if (world_model == nullptr || field_geometry == nullptr)
        return;

    painter = new QPainter{this};
    painter->translate(width() / 2, height() / 2); // bring the reference coordinate to the middle
    painter->scale(scale_ratio, -scale_ratio); // change the y axis direction (make it upward)

    draw_field_lines();
    draw_robots();
    draw_ball();

    delete painter;

}

void SoccerView::draw_robots()
{
    QColor our_color = world_model->is_yellow ? yellow_team_color : blue_team_color;
    QColor opp_color = world_model->is_yellow ? blue_team_color : yellow_team_color;

    for (const auto& robot: world_model->our)
        draw_robot(robot.pos, robot.dir, our_color);

    for (const auto& robot: world_model->opp)
        draw_robot(robot.pos, robot.dir, opp_color);
}

void SoccerView::draw_robot(const rcsc::Vector2D& pos, const rcsc::Vector2D& dir, const QColor& color)
{
    double robot_radius = knowledge::ROBOT_RADIUS;

    // draw robot body
    painter->setPen(QPen(color));
    painter->setBrush(QBrush(color));
    painter->drawEllipse((pos.x-robot_radius)*100, (pos.y-robot_radius)*100, 2*robot_radius*100, 2*robot_radius*100);

    // draw robot direction
    painter->setPen(QPen(Qt::black, 2));
    rcsc::Vector2D start{pos};
    rcsc::Vector2D end{ start + robot_radius*dir };
    painter->drawLine(start.x*100, start.y*100, end.x*100, end.y*100);
}

void SoccerView::draw_ball()
{
    double ball_radius = knowledge::BALL_RADIUS;
    painter->setPen(QPen(QColor("orangered")));
    painter->setBrush(QBrush(QColor("orangered")));
    painter->drawEllipse((world_model->ball.pos.x-ball_radius)*100, (world_model->ball.pos.y-ball_radius)*100, 2*ball_radius*100, 2*ball_radius*100);

    for (int i{1}; i < ball_trail.length(); i++)
    {
        painter->setPen(QPen(QColor("orange")));
        painter->setBrush(QBrush(QColor("orange")));
        painter->drawEllipse((ball_trail[i].x-ball_radius)*100, (ball_trail[i].y-ball_radius)*100, 2*ball_radius*100, 2*ball_radius*100);

    }
}

void SoccerView::draw_field_lines()
{
    for(const auto & line : field_geometry->field.field_lines)
    {
        painter->setPen(QPen(Qt::white, line.thickness*300)); // x300 is better for the sake of view (instead of x100)
        painter->drawLine(line.begin.x*100, line.begin.y*100, line.end.x*100, line.end.y*100);
    }
}

void SoccerView::redraw()
{
    update();
}

void SoccerView::wheelEvent ( QWheelEvent * event )
{
    if (event->delta() < 0)
        scale_ratio +=0.01;
    else if (event->delta() > 0)
        scale_ratio -=0.01;

    if (scale_ratio <= 0)
        scale_ratio = 0.01;
}

void SoccerView::worldmodel_callback (const pack_msgs::msg::WorldModel::SharedPtr msg)
{
    world_model = msg;
    ball_trail.push_front(rcsc::Vector2D{world_model->ball.pos});
    if (ball_trail.length() > 7)
        ball_trail.pop_back();
    postRedraw();
}

void SoccerView::geometry_callback (const pack_msgs::msg::SSLVisionGeometry::SharedPtr msg)
{
    field_geometry = msg;
    postRedraw();
}


