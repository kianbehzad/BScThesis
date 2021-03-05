//
// Created by Kian Behzad on 3/5/21.
//

#include "rclcpp/rclcpp.hpp"
#include "pack_ai/ai/ai_node.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AINode>(rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)));
    rclcpp::shutdown();
    return 0;
}
