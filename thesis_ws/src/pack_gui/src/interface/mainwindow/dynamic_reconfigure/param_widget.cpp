//
// Created by kian on 01.01.21.
//

#include "pack_gui/interface/mainwindow/dynamic_reconfigure/param_widget.h"

ParamWidget::ParamWidget(rclcpp::Parameter _parameter, std::string server_node_name, const std::shared_ptr<rclcpp::Node>& node, QWidget* parent) : QWidget(parent), parameter(_parameter)
{
    // define widget stuff
    main_layout = new QGridLayout{};
    name = new QLabel;
    submit = new QPushButton{};

    name->setText(QString::fromStdString(parameter.get_name()));
    submit->setText("submit");

    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
    {
        QLineEdit* temp = new QLineEdit{};
        temp->setValidator(new QIntValidator(1024, 65353));
        temp->setText(QString::number(parameter.as_int()));
        value = temp;
    }
    else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
        QLineEdit* temp = new QLineEdit{};
        temp->setValidator(new QDoubleValidator());
        temp->setText(QString::number(parameter.as_double()));
        value = temp;
    }
    else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
    {
        QLineEdit* temp = new QLineEdit{};
        temp->setText(QString::fromStdString(parameter.as_string()));
        value = temp;
    }
    else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
    {
        QCheckBox* temp = new QCheckBox{};
        temp->setText("isTrue");
        temp->setChecked(parameter.as_bool());
        value = temp;
    }

    // create remote client to the server node
    client = std::make_shared<rclcpp::AsyncParametersClient>(node, server_node_name);

    // connect submit button
    connect(this->submit, SIGNAL(pressed()), this, SLOT(submit_pressed_handle()));

    // form the widget
    main_layout->addWidget(name, 0, 0, 1, 1);
    main_layout->addWidget(value, 0, 1, 1, 1);
    main_layout->addWidget(submit, 0, 2, 1, 1);
    main_layout->setContentsMargins(0, 0, 0, 0);
    this->setContentsMargins(0, 0, 0, 0);
    this->setLayout(main_layout);

}

ParamWidget::~ParamWidget()
{
    delete name;
    delete value;
    delete submit;
    delete main_layout;
}

void ParamWidget::submit_pressed_handle()
{
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
    {
        QLineEdit* tmp = dynamic_cast<QLineEdit*>(value);
        auto results = client->set_parameters(
                {rclcpp::Parameter(parameter.get_name(), tmp->text().toInt()),});
    }
    else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
        QLineEdit* tmp = dynamic_cast<QLineEdit*>(value);
        auto results = client->set_parameters(
                {rclcpp::Parameter(parameter.get_name(), tmp->text().toDouble()),});
    }
    else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
    {
        QLineEdit* tmp = dynamic_cast<QLineEdit*>(value);
        auto results = client->set_parameters(
                {rclcpp::Parameter(parameter.get_name(), tmp->text().toStdString()),});
    }
    else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
    {
        QCheckBox* tmp = dynamic_cast<QCheckBox*>(value);
        auto results = client->set_parameters(
                {rclcpp::Parameter(parameter.get_name(), tmp->isChecked()),});
    }
}

QString ParamWidget::param_val_to_string()
{
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        return QString::number(parameter.as_int());
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        return QString::number(parameter.as_double());
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
        return QString::fromStdString(parameter.as_string());
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
        return QString(parameter.as_bool() ? "True" : "False");
    return QString("WARNING: NOT DEFINED");
}
