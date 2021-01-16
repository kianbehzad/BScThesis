//
// Created by Kian Behzad on 1/16/21.
//

#ifndef PACK_AGENT_SKILL_H
#define PACK_AGENT_SKILL_H

#include "pack_agent/agent/extern_variables.h"
#include "pack_msgs/msg/robot_command.hpp"
#include "pack_msgs/msg/skill.hpp"

class Skill
{
public:
    Skill();
    ~Skill();
    virtual pack_msgs::msg::RobotCommand execute(const pack_msgs::msg::Skill& skill) = 0;


private:

};

#endif //PACK_AGENT_SKILL_H
