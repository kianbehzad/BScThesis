//
// Created by Kian Behzad on 1/16/21.
//

#include "pack_agent/agent/skill/skill_none.h"

SkillNone::SkillNone() : Skill()
{

}

SkillNone::~SkillNone() = default;

pack_msgs::msg::RobotCommand SkillNone::execute(const pack_msgs::msg::Robot& robot, const pack_msgs::msg::Skill& skill)
{
    pack_msgs::msg::RobotCommand robot_command;
    robot_command.robot_id = robot.id;

    return robot_command;

}

