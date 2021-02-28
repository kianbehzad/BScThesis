//
// Created by Kian Behzad on 2/27/21.
//

#ifndef PACK_AGENT_SKILL_GOTOPOINT_AVOID_H
#define PACK_AGENT_SKILL_GOTOPOINT_AVOID_H

#include "pack_agent/agent/skill/skill.h"
#include "pack_agent/agent/skill/pid.h"
#include "pack_msgs/msg/skill_goto_point_avoid.hpp"

#include <QDebug>

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

    // potential field functions
    rcsc::Vector2D calculate_repulsion_classic(const pack_msgs::msg::Robot& robot,
                                       const rcsc::Vector2D& obs_center,
                                       const double& obs_radius,
                                       const double& rep_step,
                                       const double& prediction);

    rcsc::Vector2D calculate_repulsion_GNRON(const pack_msgs::msg::Robot& robot,
                                               const rcsc::Vector2D& obs_center,
                                               const rcsc::Vector2D& goal_center,
                                               const double& obs_radius,
                                               const double& rep_step,
                                               const double& prediction,
                                               const int& n);

};

#endif //PACK_AGENT_SKILL_GOTOPOINT_AVOID_H
