//
// Created by Kian Behzad on 3/5/21.
//

#include "pack_ai/ai/ai_node.h"

AINode::AINode(const rclcpp::NodeOptions & options) : Node("ai_node", options)
{
    RCLCPP_INFO(this->get_logger(), "ai node started");
}

AINode::~AINode()
{

}
