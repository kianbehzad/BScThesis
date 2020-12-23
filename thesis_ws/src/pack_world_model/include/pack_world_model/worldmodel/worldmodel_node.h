//
// Created by kian behzad on 4/2/20.
//

#ifndef PACK_WORLD_MODEL_WORLDMODEL_NODE_H
#define PACK_WORLD_MODEL_WORLDMODEL_NODE_H

#include <memory>
#include <chrono>
#include <string>
#include <functional>

#include "pack_msgs/msg/ssl_vision_detection.hpp"
#include "pack_msgs/msg/ssl_vision_geometry.hpp"
#include "pack_msgs/msg/robot_command.hpp"
#include "pack_msgs/msg/world_model.hpp"

#include "pack_world_model/worldmodel/util/config.h"
#include "pack_world_model/worldmodel/wm/worldmodel.h"

#include "pack_util/core/knowledge.h"

#include "rclcpp/rclcpp.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;


class WorldModelNode : public rclcpp::Node
{
public:
    WorldModelNode(const rclcpp::NodeOptions & options);

private:
    std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr)> params_change_callback;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub;
    void define_params_change_callback_lambda_function();

    void vision_detection_callback(const pack_msgs::msg::SSLVisionDetection::SharedPtr msg);
    rclcpp::Subscription<pack_msgs::msg::SSLVisionDetection>::SharedPtr vision_detection_subscription;

    void vision_geometry_callback(const pack_msgs::msg::SSLVisionGeometry::SharedPtr msg);
    rclcpp::Subscription<pack_msgs::msg::SSLVisionGeometry>::SharedPtr vision_geometry_subscription;

    void command_callback(const pack_msgs::msg::RobotCommand::SharedPtr msg);
    rclcpp::Subscription<pack_msgs::msg::RobotCommand>::SharedPtr command_subscription[knowledge::MAX_ROBOT_NUM];

    rclcpp::Publisher<pack_msgs::msg::WorldModel>::SharedPtr worldmodel_publisher;

    std::shared_ptr<WorldModel> wm;
    int frame, packs;
};



#endif //PACK_WORLD_MODEL_WORLDMODEL_NODE_H
