//
// Created by Kian Behzad on 1/19/21.
//

#include "rclcpp/rclcpp.hpp"
#include "pack_world_model/simple_worldmodel/simple_worldmodel_node.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleWorldModelNode>(rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)));
    rclcpp::shutdown();
    return 0;
}