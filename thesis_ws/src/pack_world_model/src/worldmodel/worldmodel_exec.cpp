//
// Created by kian behzad on 4/2/20.
//
#include "rclcpp/rclcpp.hpp"
#include "pack_world_model/worldmodel/worldmodel_node.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WorldModelNode>(rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)));
    rclcpp::shutdown();
    return 0;
}

