//
// Created by Kian Behzad on 1/15/21.
//
#include "rclcpp/rclcpp.hpp"
#include "pack_agent/agent/agent_node.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AgentNode>(rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)));
    rclcpp::shutdown();
    return 0;
}

