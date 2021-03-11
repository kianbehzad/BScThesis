//
// Created by Kian Behzad on 1/16/21.
//

#ifndef PACK_AGENT_SKILL_GOTOPOINT_H
#define PACK_AGENT_SKILL_GOTOPOINT_H

#include "pack_agent/agent/skill/skill.h"
#include "pack_agent/agent/skill/control_tools.h"
#include "pack_msgs/msg/skill_goto_point.hpp"

#include <QDebug>

using control_tool::PID;

class SkillGotoPoint : public Skill
{
public:
    SkillGotoPoint();
    ~SkillGotoPoint();
    pack_msgs::msg::RobotCommand execute(const pack_msgs::msg::Robot& robot, const pack_msgs::msg::Skill& skill);

private:
    pack_msgs::msg::SkillGotoPoint skill_gotopoint_msg;
    PID* pos_pid;
    PID* angle_pid;

};

#endif //PACK_AGENT_SKILL_GOTOPOINT_H
