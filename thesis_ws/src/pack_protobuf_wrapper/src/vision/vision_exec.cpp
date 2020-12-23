//
// Created by kian behzad on 3/22/20.
//

#include "rclcpp/rclcpp.hpp"
#include "pack_protobuf_wrapper/vision/vision_node.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionNode>(rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)));
    rclcpp::shutdown();
    return 0;
}

