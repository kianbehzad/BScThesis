//
// Created by Kian Behzad on 3/5/21.
//

#ifndef PACK_UTIL_SKILL_HANDLER_H
#define PACK_UTIL_SKILL_HANDLER_H

#include "pack_msgs/msg/skill.hpp"
#include "pack_util/geom/vector_2d.h"
#include <QMap>

class SkillHandler
{
public:
    SkillHandler();
    ~SkillHandler();

    QMap<int, pack_msgs::msg::Skill> get_skills();

    void direct_velocity(const int &id, const rcsc::Vector2D& velocity, const rcsc::Vector2D& look_at = rcsc::Vector2D{}.invalidate());
    void gotopoint(const int &id, const rcsc::Vector2D& destination, const rcsc::Vector2D& look_at = rcsc::Vector2D{}.invalidate());
    void gotopoint_avoid(const int &id, const rcsc::Vector2D& destination, const rcsc::Vector2D& look_at = rcsc::Vector2D{}.invalidate(),
                         const bool& consider_ball_as_obstacle = false,
                         const bool& consider_our_robot_as_obstacle = false,
                         const bool& consider_opp_robot_as_obstacle = false,
                         const bool& consider_our_penalty_as_obstacle = false,
                         const bool& consider_opp_penalty_as_obstacle = false);


private:
    QMap<int, pack_msgs::msg::Skill> skills;
};

#endif //PACK_UTIL_SKILL_HANDLER_H
