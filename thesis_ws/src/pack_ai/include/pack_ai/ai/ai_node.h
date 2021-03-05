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

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class AINode : public rclcpp::Node
{
public:
    AINode(const rclcpp::NodeOptions & options);
    ~AINode();

private:


};

#endif //PACK_AI_AI_NODE_H
