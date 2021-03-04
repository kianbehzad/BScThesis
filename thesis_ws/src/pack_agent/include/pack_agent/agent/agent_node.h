//
// Created by Kian Behzad on 1/15/21.
//

#ifndef PACK_AGENT_AGENT_NODE_H
#define PACK_AGENT_AGENT_NODE_H

#include <memory>
#include <chrono>
#include <string>
#include <functional>

#include <QDebug>

#include "pack_agent/agent/extern_variables.h"
#include "pack_agent/agent/skill/skill.h"
#include "pack_agent/agent/skill/skill_gotopoint.h"
#include "pack_agent/agent/skill/skill_gotopoint_avoid.h"
#include "pack_agent/agent/skill/skill_direct_velocity.h"
#include "pack_agent/agent/skill/skill_none.h"
#include "pack_util/core/drawer.h"
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
    ~AgentNode();

private:
    // parameter client
    std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr)> params_change_callback;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub;
    void define_params_change_callback_lambda_function();

    // world model subscription
    void worldmodel_callback(const pack_msgs::msg::WorldModel::SharedPtr msg);
    rclcpp::Subscription<pack_msgs::msg::WorldModel>::SharedPtr worldmodel_subscription;

    // skill subscription
    void skill_callback(const pack_msgs::msg::Skill::SharedPtr msg);
    rclcpp::Subscription<pack_msgs::msg::Skill>::SharedPtr skill_subscription;
    pack_msgs::msg::Skill::SharedPtr skill_msg;

    // robot command publisher
    rclcpp::Publisher<pack_msgs::msg::RobotCommand>::SharedPtr robotcommand_publisher;

    // debug draws publisher
    rclcpp::Publisher<pack_msgs::msg::Shapes>::SharedPtr debugdraws_publisher;

    // Skills
    Skill* skill;
    SkillGotoPoint* skill_gotopoint;
    SkillGotoPointAvoid* skill_gotopoint_avoid;
    SkillDirectVelocity* skill_direct_velocity;
    SkillNone* skill_none;

};

#endif //PACK_AGENT_AGENT_NODE_H
