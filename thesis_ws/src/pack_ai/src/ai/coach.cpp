//
// Created by Kian Behzad on 3/13/21.
//

#include "pack_ai/ai/coach.h"

Coach::Coach() = default;

Coach::~Coach() = default;

void Coach::execute()
{
//    push_ball(0, field.oppGoal());
    defense_at_penalty(0);



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

void Coach::defense_at_penalty(const int& id)
{
    rcsc::Vector2D ball_vel = extern_wm->ball.vel;
    rcsc::Vector2D ball_pos = extern_wm->ball.pos + ball_vel*0.1;
    rcsc::Segment2D ball_path{ball_pos, ball_pos+ball_vel*10};
    rcsc::Segment2D right_penalty_area_seg{field.ourPenaltyRect().topRight(), field.ourPenaltyRect().bottomRight()};
    rcsc::Rect2D left_of_penalty_rect{field.ourCornerL(), field.ourPenaltyRect().topRight()};
    rcsc::Rect2D right_of_penalty_rect{field.ourCornerR(), field.ourPenaltyRect().bottomRight()};

    rcsc::Vector2D position;
    QList<rcsc::Vector2D> sols = field.ourPAreaIntersect(ball_path);
    if (sols.empty())
        if (left_of_penalty_rect.contains(ball_pos)) position = rcsc::Vector2D{ball_pos.x, field.ourPenaltyRect().top()};
        else if (right_of_penalty_rect.contains(ball_pos)) position = rcsc::Vector2D{ball_pos.x, field.ourPenaltyRect().bottom()};
        else position = right_penalty_area_seg.nearestPoint(ball_pos);
    else if (sols.size() == 1) position = sols[0];
    else position = (ball_pos.dist(sols[0]) > ball_pos.dist(sols[1])) ? sols[1] : sols[0];
    extern_skill_handler->gotopoint_avoid(id, position, ball_pos, false, true, false, true, false);
}


