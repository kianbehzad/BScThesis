//
// Created by Kian Behzad on 1/16/21.
//

#include "pack_agent/agent/skill/skill_gotopoint.h"

SkillGotoPoint::SkillGotoPoint() : Skill()
{

}

SkillGotoPoint::~SkillGotoPoint() = default;

pack_msgs::msg::RobotCommand SkillGotoPoint::execute(const pack_msgs::msg::Skill& skill)
{
    skill_gotopoint_msg = skill.skill_gotopoint;
    id = skill.id;
    pack_msgs::msg::RobotCommand robot_command;

    qDebug() << "heyy:  " << id;
    return robot_command;

}