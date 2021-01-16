//
// Created by Kian Behzad on 1/15/21.
//

#ifndef PACK_AGENT_AGENT_NODE_H
#define PACK_AGENT_AGENT_NODE_H

#include <memory>
#include <chrono>
#include <string>
#include <functional>

#include "pack_agent/agent/extern_variables.h"
#include "pack_msgs/msg/robot_command.hpp"
#include "pack_msgs/msg/world_model.hpp"
#include "pack_msgs/msg/skill.hpp"
#include "pack_msgs/msg/skill_goto_point.hpp"
#include "pack_msgs/msg/robot_command.hpp"

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class AgentNode : public rclcpp::Node
{
public:
    AgentNode(const rclcpp::NodeOptions & options);

private:
    // parameter client
    std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr)> params_change_callback;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub;
    void define_params_change_callback_lambda_function();
    // prameter variables
    double P_vel = 10;
    double I_vel = 10;
    double D_vel = 10;
    double P_pos = 10;
    double I_pos = 10;
    double D_pos = 10;
    double max_vel = 4;

    // world model subscription
    void worldmodel_callback(const pack_msgs::msg::WorldModel::SharedPtr msg);
    rclcpp::Subscription<pack_msgs::msg::WorldModel>::SharedPtr worldmodel_subscription;

    // skill subscription
    void skill_callback(const pack_msgs::msg::Skill::SharedPtr msg);
    rclcpp::Subscription<pack_msgs::msg::Skill>::SharedPtr skill_subscription;
    pack_msgs::msg::Skill::SharedPtr skill;

    // robot command publisher
    rclcpp::Publisher<pack_msgs::msg::RobotCommand>::SharedPtr robotcommand_publisher;

};

#endif //PACK_AGENT_AGENT_NODE_H
