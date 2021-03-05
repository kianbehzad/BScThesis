//
// Created by Kian Behzad on 1/3/21.
//

#include "rclcpp/rclcpp.hpp"
#include "pack_msgs/msg/world_model.hpp"
#include "pack_msgs/msg/ssl_vision_geometry.hpp"
#include "pack_msgs/msg/shapes.hpp"
#include "pack_util/geom/vector_2d.h"
#include "pack_util/core/knowledge.h"

#include <QOpenGLWidget>
#include <QDebug>
#include <QPainter>
#include <QPen>
#include <QWheelEvent>
#include <QMap>
#include <QRadialGradient>

#include <memory>
#include <chrono>

using namespace std::chrono_literals;

#ifndef PACK_GUI_SOCCER_VIEW_H
#define PACK_GUI_SOCCER_VIEW_H

#define CVTY( y ) ( -(y)*100 )  //convert received message's y coordinate to OPENGL coordinate
#define CVTX( x ) ( (x)*100 )   //convert received message's x coordinate to OPENGL coordinate
#define CVTM( m ) ( (m)*100 )   //convert received message's magnitude to OPENGL magnitude

struct NodeInThread
{
    NodeInThread(std::shared_ptr<rclcpp::Node> _node) : node(_node)
    {}
    void operator()(){
        rclcpp::spin(node);
        rclcpp::shutdown();
    }

    std::shared_ptr<rclcpp::Node> node;
};

class SoccerView : public QOpenGLWidget
{
    Q_OBJECT
public:
    SoccerView(QWidget *parent = 0);
    ~SoccerView();

private:
    //draws
    QPainter* painter;
    double scale_ratio;
    QList<rcsc::Vector2D> ball_trail;
    QColor blue_team_color, yellow_team_color;
    void draw_ball();
    void draw_field_lines();
    void draw_robot(const int& id, const rcsc::Vector2D& pos, const rcsc::Vector2D& dir, const QColor& color);
    void draw_robots();
    void draw_debug_draws();

    // ros2 stuff
    std::shared_ptr<rclcpp::Node> node;
    pack_msgs::msg::WorldModel::SharedPtr world_model;
    pack_msgs::msg::SSLVisionGeometry::SharedPtr field_geometry;
    QMap<QString, pack_msgs::msg::Shapes::SharedPtr>  debug_draws;

    void worldmodel_callback (const pack_msgs::msg::WorldModel::SharedPtr msg);
    rclcpp::Subscription<pack_msgs::msg::WorldModel>::SharedPtr worldmodel_subscription;

    void geometry_callback (const pack_msgs::msg::SSLVisionGeometry::SharedPtr msg);
    rclcpp::Subscription<pack_msgs::msg::SSLVisionGeometry>::SharedPtr geometry_subscription;

    void debug_draw_callback (const pack_msgs::msg::Shapes ::SharedPtr msg);
    rclcpp::Subscription<pack_msgs::msg::Shapes>::SharedPtr debug_draw_subscription;

    bool active_soccer_view; //ros2 param
    bool draw_debugs;        //ros2 param
    std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr)> params_change_callback;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub;
    void define_params_change_callback_lambda_function();

protected:
    void paintGL () override;
    void initializeGL() override;
    void wheelEvent ( QWheelEvent * event );

private slots:
    void redraw();
signals:
    void postRedraw();

};

#endif //PACK_GUI_SOCCER_VIEW_H
