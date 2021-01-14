//
// Created by kian on 31.12.20.
//
#include "pack_gui/interface/mainwindow/dynamic_reconfigure/dynamic_reconfigure.h"

DynamicReconfigure::DynamicReconfigure(QWidget* parent) : QTabWidget(parent)
{
    // find param file
    for (int i{}; i < extern_argv.size(); i++)
        if(extern_argv[i] == "--params-file")
            param_file_path = QString::fromStdString(extern_argv[i+1]);

    if(param_file_path.isEmpty())
        return; // TODO display a message on gui

    // parse param file
    rcutils_allocator_t alocator = rcutils_get_default_allocator();
    rcl_params_t *param_st = rcl_yaml_node_struct_init(alocator);
    rcl_parse_yaml_file(param_file_path.toStdString().c_str(), param_st);
    parsed = rclcpp::parameter_map_from(param_st);

    // debug - print gathered information from param file
//    for( const auto& pair : parsed )
//    {
//        qDebug() << QString::fromStdString(pair.first);
//        for(const auto& val : pair.second)
//            qDebug() << "   " << QString::fromStdString(val.get_name()) << " - " << QString::fromStdString(val.get_type_name());
//    }

    // create dynamic-reconfigure node
    node = rclcpp::Node::make_shared("dynamic_reconfigure_node");

    // create the widget
    for(const auto& pair : parsed)
    {
        tab_widget_list.push_back(new QScrollArea{});
        QWidget* tmp_widget = new QWidget{};
        QVBoxLayout* tmp_layout = new QVBoxLayout{};
        for(const auto& val : pair.second)
            tmp_layout->addWidget(new ParamWidget{val, pair.first, node});
        tmp_widget->setLayout(tmp_layout);
        tab_widget_list.last()->setWidget(tmp_widget);
        this->addTab(tab_widget_list.last(), QString::fromStdString(pair.first));
    }


}

DynamicReconfigure::~DynamicReconfigure()
{
    for (auto& widg : tab_widget_list)
    {
        QLayoutItem *child;
        while ((child = widg->widget()->layout()->takeAt(0)) != 0)
            delete child;
        delete widg->widget()->layout();
        delete widg->widget();
        delete widg;
    }

}