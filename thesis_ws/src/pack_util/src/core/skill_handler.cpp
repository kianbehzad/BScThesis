//
// Created by Kian Behzad on 3/5/21.
//

#include "pack_util/core/skill_handler.h"


SkillHandler::SkillHandler()
{

}

SkillHandler::~SkillHandler()
{
    skills.clear();
}

QMap<int, pack_msgs::msg::Skill> SkillHandler::get_skills()
{
    QMap<int, pack_msgs::msg::Skill> tmp = skills;
    skills.clear();
    return tmp;
}

void SkillHandler::direct_velocity(const int &id, const rcsc::Vector2D& velocity, const rcsc::Vector2D& look_at)
{
    pack_msgs::msg::Skill skill;
    skill.id = id;
    skill.skill_type = pack_msgs::msg::Skill::SKILLDIRECTVELOCITY;
    skill.skill_direct_velocity.velocity = velocity.toParsianMessage();
    skill.skill_direct_velocity.look_at = look_at.toParsianMessage();
    skills[id] = skill;
}

void SkillHandler::gotopoint(const int &id, const rcsc::Vector2D& destination, const rcsc::Vector2D& look_at)
{
    pack_msgs::msg::Skill skill;
    skill.id = id;
    skill.skill_type = pack_msgs::msg::Skill::SKILLGOTOPOINT;
    skill.skill_gotopoint.destination = destination.toParsianMessage();
    skill.skill_gotopoint.look_at = look_at.toParsianMessage();
    skills[id] = skill;
}

void SkillHandler::gotopoint_avoid(const int &id, const rcsc::Vector2D& destination, const rcsc::Vector2D& look_at,
                     const bool& consider_ball_as_obstacle,
                     const bool& consider_our_robot_as_obstacle,
                     const bool& consider_opp_robot_as_obstacle,
                     const bool& consider_our_penalty_as_obstacle,
                     const bool& consider_opp_penalty_as_obstacle)
{
    pack_msgs::msg::Skill skill;
    skill.id = id;
    skill.skill_type = pack_msgs::msg::Skill::SKILLGOTOPOINTAVOID;
    skill.skill_gotopoint_avoid.destination = destination.toParsianMessage();
    skill.skill_gotopoint_avoid.look_at = look_at.toParsianMessage();
    skill.skill_gotopoint_avoid.consider_ball_as_obstacle = consider_ball_as_obstacle;
    skill.skill_gotopoint_avoid.consider_our_robot_as_obstacle = consider_our_robot_as_obstacle;
    skill.skill_gotopoint_avoid.consider_opp_robot_as_obstacle = consider_opp_robot_as_obstacle;
    skill.skill_gotopoint_avoid.consider_our_penalty_as_obstacle = consider_our_penalty_as_obstacle;
    skill.skill_gotopoint_avoid.consider_opp_penalty_as_obstacle = consider_opp_penalty_as_obstacle;
    skills[id] = skill;
}
