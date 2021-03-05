//
// Created by Kian Behzad on 1/16/21.
//

#ifndef PACK_AGENT_SKILL_NONE_H
#define PACK_AGENT_SKILL_NONE_H

#include "pack_agent/agent/skill/skill.h"

#include <QDebug>

class SkillNone : public Skill
{
public:
    SkillNone();
    ~SkillNone();
    pack_msgs::msg::RobotCommand execute(const pack_msgs::msg::Skill& skill);

private:
    int id;

};
#endif //PACK_AGENT_SKILL_NONE_H
