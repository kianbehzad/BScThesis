//
// Created by Kian Behzad on 1/15/21.
//

#ifndef PACK_AGENT_AGENT_NODE_H
#define PACK_AGENT_AGENT_NODE_H

#include <memory>
#include <chrono>
#include <string>
#include <functional>

#include "pack_msgs/msg/robot_command.hpp"
#include "pack_msgs/msg/world_model.hpp"

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class AgentNode : public rclcpp::Node
{
public:
    AgentNode(const rclcpp::NodeOptions & options);

private:
};

#endif //PACK_AGENT_AGENT_NODE_H
