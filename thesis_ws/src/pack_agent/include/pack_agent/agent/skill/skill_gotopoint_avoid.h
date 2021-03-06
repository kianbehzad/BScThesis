//
// Created by Kian Behzad on 2/27/21.
//

#ifndef PACK_AGENT_SKILL_GOTOPOINT_AVOID_H
#define PACK_AGENT_SKILL_GOTOPOINT_AVOID_H

#include "pack_agent/agent/skill/skill.h"
#include "pack_agent/agent/skill/control_tools.h"
#include "pack_util/geom/segment_2d.h"
#include "pack_msgs/msg/skill_goto_point_avoid.hpp"

#include <QDebug>

using control_tool::PID;

class SkillGotoPointAvoid : public Skill
{
public:
    SkillGotoPointAvoid();
    ~SkillGotoPointAvoid();
    pack_msgs::msg::RobotCommand execute(const pack_msgs::msg::Skill& skill);


private:
    pack_msgs::msg::SkillGotoPointAvoid skill_gotopointavoid_msg;
    int id;
    PID* angle_pid;
    int local_minimum_counter;

};

#endif //PACK_AGENT_SKILL_GOTOPOINT_AVOID_H
