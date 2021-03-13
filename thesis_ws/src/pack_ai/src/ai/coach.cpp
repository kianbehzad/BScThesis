//
// Created by Kian Behzad on 3/13/21.
//

#include "pack_ai/ai/coach.h"

Coach::Coach() = default;

Coach::~Coach() = default;

void Coach::execute()
{
    push_ball(0, field.oppGoal());
}

int Coach::ID(int id)
{
    for(int i{}; i < static_cast<int>(extern_wm->our.size()); i++)
        if (extern_wm->our[i].id == id)
            return i;
    return -1;
}

void Coach::push_ball(const int& id, const rcsc::Vector2D& destination)
{
    rcsc::Vector2D robot_pos = extern_wm->our[ID(id)].pos;
    rcsc::Vector2D ball_pos = extern_wm->ball.pos + extern_wm->ball.vel*0.3;
    rcsc::Vector2D ball_destination = (ball_pos - destination).norm();
    rcsc::Vector2D position = ball_pos + ball_destination*(knowledge::ROBOT_RADIUS + 0.05);
    Segment2D seg{robot_pos, position};
    if (robot_pos.dist(position) < 0.1)
        extern_skill_handler->direct_velocity(id, -ball_destination*0.5, destination);
    else if (seg.dist(ball_pos) < 0.09)
        extern_skill_handler->gotopoint_avoid(id, position, destination, true);
    else
        extern_skill_handler->gotopoint(id, position, destination);
}

