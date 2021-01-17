//
// Created by Kian Behzad on 1/17/21.
//
#include "rclcpp/rclcpp.hpp"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rclcpp::Node>("agent_param_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)));
    rclcpp::shutdown();
    return 0;
}
