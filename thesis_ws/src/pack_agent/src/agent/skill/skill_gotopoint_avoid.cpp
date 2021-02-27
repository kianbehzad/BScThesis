//
// Created by Kian Behzad on 2/27/21.
//

#include "pack_agent/agent/skill/skill_gotopoint_avoid.h"

SkillGotoPointAvoid::SkillGotoPointAvoid() : Skill()
{

}

SkillGotoPointAvoid::~SkillGotoPointAvoid()
{

}

pack_msgs::msg::RobotCommand SkillGotoPointAvoid::execute(const pack_msgs::msg::Skill& skill)
{
    pack_msgs::msg::RobotCommand robot_command;

    return robot_command;
}