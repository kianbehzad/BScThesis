//
// Created by ubuntu on 31.12.20.
//

#ifndef PACK_GUI_DYNAMIC_RECONFIGURE_H
#define PACK_GUI_DYNAMIC_RECONFIGURE_H

#include <memory>

#include "rcl_yaml_param_parser/parser.h"
#include "rclcpp/parameter_map.hpp"
#include "pack_gui/interface/extern_variables.h"
#include "pack_gui/interface/mainwindow/dynamic_reconfigure/param_widget.h"

#include <QTabWidget>
#include <QScrollArea>
#include <QWidget>
#include <QString>
#include <QDebug>
#include <QList>
#include <QVBoxLayout>

class DynamicReconfigure : public QTabWidget
{
    Q_OBJECT
public:
    DynamicReconfigure(QWidget* parent = 0);
    ~DynamicReconfigure();

private:
    // parse param file
    QString param_file_path;
    std::unordered_map<std::string, std::vector<rclcpp::Parameter>> parsed;

    // change parameter node
    std::shared_ptr<rclcpp::Node> node;

    // widget stuff
    QList<QScrollArea*> tab_widget_list;
};

#endif //PACK_GUI_DYNAMIC_RECONFIGURE_H
