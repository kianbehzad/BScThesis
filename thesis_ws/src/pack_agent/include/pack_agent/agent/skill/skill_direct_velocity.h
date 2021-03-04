//
// Created by Kian Behzad on 1/16/21.
//

#ifndef PACK_AGENT_DIRECT_VELOCITY_H
#define PACK_AGENT_DIRECT_VELOCITY_H

#include "pack_agent/agent/skill/skill.h"
#include "pack_agent/agent/skill/control_tools.h"
#include "pack_msgs/msg/skill_direct_velocity.hpp"

#include <QDebug>

using control_tool::PID;

class SkillDirectVelocity : public Skill
{
public:
    SkillDirectVelocity();
    ~SkillDirectVelocity();
    pack_msgs::msg::RobotCommand execute(const pack_msgs::msg::Skill& skill);

private:
    pack_msgs::msg::SkillDirectVelocity skill_direct_velocity_msg;
    int id;
    PID* angle_pid;

};

#endif //PACK_AGENT_DIRECT_VELOCITY_H
