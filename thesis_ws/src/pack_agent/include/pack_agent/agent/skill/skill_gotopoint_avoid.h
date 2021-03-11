//
// Created by Kian Behzad on 2/27/21.
//

#ifndef PACK_AGENT_SKILL_GOTOPOINT_AVOID_H
#define PACK_AGENT_SKILL_GOTOPOINT_AVOID_H

#include "pack_agent/agent/skill/skill.h"
#include "pack_agent/agent/skill/control_tools.h"
#include "pack_util/geom/segment_2d.h"
#include "pack_util/core/field.h"
#include "pack_msgs/msg/skill_goto_point_avoid.hpp"

#include <QDebug>

using control_tool::PID;

struct Obstacle
{
    explicit Obstacle(const pack_msgs::msg::Robot& _obj = pack_msgs::msg::Robot{}) : obj{_obj}{}
    pack_msgs::msg::Robot obj;
    double repulsion_static_radius      = extern_repulsion_static_radius;
    double repulsion_static_step        = extern_repulsion_static_step;
    double repulsion_static_prediction  = extern_repulsion_static_prediction;
    double repulsion_dynamic_step       = extern_repulsion_dynamic_step;
    double repulsion_dynamic_prediction = extern_repulsion_dynamic_prediction;
    int GNRON_degree                    = 10;
};

class SkillGotoPointAvoid : public Skill
{
public:
    SkillGotoPointAvoid();
    ~SkillGotoPointAvoid();
    pack_msgs::msg::RobotCommand execute(const pack_msgs::msg::Robot& robot, const pack_msgs::msg::Skill& skill);


private:
    CField field;
    pack_msgs::msg::SkillGotoPointAvoid skill_gotopointavoid_msg;
    PID* angle_pid;
    int local_minimum_counter;

};

#endif //PACK_AGENT_SKILL_GOTOPOINT_AVOID_H
