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
    node = rclcpp::Node::make_shared("soccer_view_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // set up parameter client
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);
    while (!parameters_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the parameter service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(node->get_logger(), "parameter service not available, waiting again...");
    }

    active_soccer_view = parameters_client->get_parameter("active_soccer_view", active_soccer_view);
    draw_debugs = parameters_client->get_parameter("draw_debugs", draw_debugs);

    // set up parameter-change callback
    define_params_change_callback_lambda_function();
    parameter_event_sub = parameters_client->on_parameter_event(params_change_callback);

    // worldmodel callback
    worldmodel_subscription = node->create_subscription<pack_msgs::msg::WorldModel>("/world_model", 10, std::bind(&SoccerView::worldmodel_callback, this, std::placeholders::_1));
    // geometry callback
    geometry_subscription = node->create_subscription<pack_msgs::msg::SSLVisionGeometry>("/vision_geom", 10, std::bind(&SoccerView::geometry_callback, this, std::placeholders::_1));
    // debug draws callback
    debug_draw_subscription = node->create_subscription<pack_msgs::msg::Shapes>("/debug_draws", 10, std::bind(&SoccerView::debug_draw_callback, this, std::placeholders::_1));

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
    painter->scale(scale_ratio, scale_ratio);

//    painter->drawLine(CVTX(0), CVTY(0), CVTX(1), CVTY(2));
//    painter->drawRect(CVTX(1), CVTY(-1), 100, 100);

    draw_field_lines();
    draw_robots();
    draw_ball();
    if(draw_debugs) draw_debug_draws();

    delete painter;

}

void SoccerView::draw_robots()
{
    QColor our_color = world_model->is_yellow ? yellow_team_color : blue_team_color;
    QColor opp_color = world_model->is_yellow ? blue_team_color : yellow_team_color;

    for (const auto& robot: world_model->our)
        draw_robot(robot.id, robot.pos, robot.dir, our_color);

    for (const auto& robot: world_model->opp)
        draw_robot(robot.id, robot.pos, robot.dir, opp_color);
}

void SoccerView::draw_robot(const int& id, const rcsc::Vector2D& pos, const rcsc::Vector2D& dir, const QColor& color)
{
    double robot_radius = knowledge::ROBOT_RADIUS;

    // draw robot body
    painter->setPen(QPen(color));
    painter->setBrush(QBrush(color));
    painter->drawEllipse(CVTX(pos.x-robot_radius), CVTY(pos.y+robot_radius), CVTM(2*robot_radius), CVTM(2*robot_radius));

    // draw robot direction
    painter->setPen(QPen(Qt::black, 2));
    rcsc::Vector2D start{pos};
    rcsc::Vector2D end{ start + robot_radius*dir };
    painter->drawLine(CVTX(start.x), CVTY(start.y), CVTX(end.x), CVTY(end.y));

    // draw robot ID
    painter->drawText(CVTX(pos.x-robot_radius), CVTY(pos.y+1.5*robot_radius), QString::number(id));
}

void SoccerView::draw_ball()
{
    double ball_radius = knowledge::BALL_RADIUS;
    painter->setPen(QPen(QColor("orangered")));
    painter->setBrush(QBrush(QColor("orangered")));
    painter->drawEllipse(CVTX(world_model->ball.pos.x-ball_radius), CVTY(world_model->ball.pos.y+ball_radius), CVTM(2*ball_radius), CVTM(2*ball_radius));

    for (int i{1}; i < ball_trail.length(); i++)
    {
        painter->setPen(QPen(QColor("orange")));
        painter->setBrush(QBrush(QColor("orange")));
        painter->drawEllipse(CVTX(ball_trail[i].x-ball_radius), CVTY(ball_trail[i].y+ball_radius), CVTM(2*ball_radius), CVTM(2*ball_radius));

    }
}

void SoccerView::draw_field_lines()
{
    for(const auto & line : field_geometry->field.field_lines)
    {
        painter->setPen(QPen(Qt::white, CVTM(line.thickness)*3)); // x3 makes it look better in the gui
        painter->drawLine(CVTX(line.begin.x), CVTY(line.begin.y), CVTX(line.end.x), CVTY(line.end.y));
    }
}

void SoccerView::draw_debug_draws()
{
    painter->setPen(QPen(Qt::black));
    painter->setBrush(Qt::NoBrush);
    for(const auto& value : debug_draws)
    {
        for(const auto& point : value->points)
        {
            painter->setPen(QPen(Qt::black, 10));
            painter->drawPoint(CVTX(point.x), CVTY(point.y));
        }

        for(const auto& rect : value->rects)
        {
            QColor color {QString::fromStdString(rect.color)};
            painter->setPen(QPen(color, 2));
            if(rect.fill) painter->setBrush(QBrush(color));
            else painter->setBrush(Qt::NoBrush);

            painter->drawRect(CVTX(rect.x), CVTY(rect.y), CVTM(rect.width), CVTM(rect.height));
        }

        for(const auto& triangle : value->triangles)
        {
            QColor color {QString::fromStdString(triangle.color)};
            painter->setPen(QPen(color, 2));
            if(triangle.fill) painter->setBrush(QBrush(color));
            else painter->setBrush(Qt::NoBrush);

            QPointF points[3] = {
                    QPointF(CVTX(triangle.x1), CVTY(triangle.y1)),
                    QPointF(CVTX(triangle.x2), CVTY(triangle.y2)),
                    QPointF(CVTX(triangle.x3), CVTY(triangle.y3))
            };
            painter->drawPolygon(points, 3);
        }

        for(const auto& line : value->lines)
        {
            QColor color {QString::fromStdString(line.color)};
            painter->setPen(QPen(color, 2));
            painter->setBrush(Qt::NoBrush);

            painter->drawLine(CVTX(line.x1), CVTY(line.y1), CVTX(line.x2), CVTY(line.y2));
        }
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
    if(!active_soccer_view) return;

    world_model = msg;
    ball_trail.push_front(rcsc::Vector2D{world_model->ball.pos});
    if (ball_trail.length() > 7)
        ball_trail.pop_back();
    postRedraw();
}

void SoccerView::geometry_callback (const pack_msgs::msg::SSLVisionGeometry::SharedPtr msg)
{
    if(!active_soccer_view) return;
    field_geometry = msg;
}

void SoccerView::debug_draw_callback (const pack_msgs::msg::Shapes::SharedPtr msg)
{
    if(!active_soccer_view) return;
    // storing draws on seperate key-values for each publisher to avoid glimpses
    debug_draws[QString::fromStdString(msg->publisher_name)] = msg;
}

void SoccerView::define_params_change_callback_lambda_function()
{
    params_change_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
        for (auto & new_parameter : event->new_parameters) {
            //do stuff
        }
        for (auto & changed_parameter : event->changed_parameters) {
            if(changed_parameter.name == "active_soccer_view")
            {
                active_soccer_view = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "draw_debugs")
            {
                draw_debugs = changed_parameter.value.bool_value;
            }
        }
        for (auto & deleted_parameter : event->deleted_parameters) {
            //do stuff
        }
    };

}


