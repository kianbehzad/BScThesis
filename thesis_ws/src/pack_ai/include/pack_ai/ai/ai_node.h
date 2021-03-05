//
// Created by Kian Behzad on 3/5/21.
//

#ifndef PACK_AI_AI_NODE_H
#define PACK_AI_AI_NODE_H

#include <memory>
#include <chrono>
#include <string>
#include <functional>

#include <QDebug>

#include "pack_ai/ai/extern_variables.h"
#include "pack_msgs/msg/world_model.hpp"

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class AINode : public rclcpp::Node
{
public:
    AINode(const rclcpp::NodeOptions & options);
    ~AINode();

private:
    // parameter client
    std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr)> params_change_callback;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub;
    void define_params_change_callback_lambda_function();

    // world model subscription
    void worldmodel_callback(const pack_msgs::msg::WorldModel::SharedPtr msg);
    rclcpp::Subscription<pack_msgs::msg::WorldModel>::SharedPtr worldmodel_subscription;


};

#endif //PACK_AI_AI_NODE_H
