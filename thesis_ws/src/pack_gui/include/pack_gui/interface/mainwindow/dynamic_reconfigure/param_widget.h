//
// Created by ubuntu on 01.01.21.
//

#ifndef PACK_GUI_PARAM_WIDGET_H
#define PACK_GUI_PARAM_WIDGET_H

#include <memory>

#include "rclcpp/parameter_map.hpp"
#include "rclcpp/rclcpp.hpp"

#include <QWidget>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QIntValidator>
#include <QDoubleValidator>
#include <QPushButton>
#include <QWidget>
#include <QString>
#include <QCheckBox>
#include <QDebug>

class ParamWidget : public QWidget
{
    Q_OBJECT
public:
    ParamWidget(rclcpp::Parameter _parameter, std::string server_node_name, const std::shared_ptr<rclcpp::Node>& node, QWidget* parent = 0);
    ~ParamWidget();

private:
    rclcpp::Parameter parameter;
    QString param_val_to_string();

    // change parameter stuff
    std::shared_ptr<rclcpp::AsyncParametersClient> client;

    // widget stuff
    QGridLayout* main_layout;
    QLabel* name;
    QWidget* value;
    QPushButton* submit;

public slots:
    void submit_pressed_handle();

};

#endif //PACK_GUI_PARAM_WIDGET_H
