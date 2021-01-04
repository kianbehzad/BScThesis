//
// Created by Kian Behzad on 1/3/21.
//

#include "rclcpp/rclcpp.hpp"
#include "pack_msgs/msg/world_model.hpp"
#include "pack_msgs/msg/ssl_vision_geometry.hpp"
#include "pack_util/geom/vector_2d.h"
#include "pack_util/core/knowledge.h"

#include <QOpenGLWidget>
#include <QDebug>
#include <QPainter>
#include <QPen>
#include <QWheelEvent>

#include <memory>

#ifndef PACK_GUI_SOCCER_VIEW_H
#define PACK_GUI_SOCCER_VIEW_H

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
    void draw_robots();

    // ros2 stuff
    std::shared_ptr<rclcpp::Node> node;
    pack_msgs::msg::WorldModel::SharedPtr world_model;
    pack_msgs::msg::SSLVisionGeometry::SharedPtr field_geometry;

    void worldmodel_callback (const pack_msgs::msg::WorldModel::SharedPtr msg);
    rclcpp::Subscription<pack_msgs::msg::WorldModel>::SharedPtr worldmodel_subscription;

    void geometry_callback (const pack_msgs::msg::SSLVisionGeometry::SharedPtr msg);
    rclcpp::Subscription<pack_msgs::msg::SSLVisionGeometry>::SharedPtr geometry_subscription;

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
