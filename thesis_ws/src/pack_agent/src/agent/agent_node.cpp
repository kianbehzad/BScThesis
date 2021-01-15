//
// Created by Kian Behzad on 1/15/21.
//

#include "pack_agent/agent/agent_node.h"

AgentNode::AgentNode(const rclcpp::NodeOptions & options) : Node("agent_node", options)
{
    RCLCPP_INFO(this->get_logger(), "hello from agent node");
}