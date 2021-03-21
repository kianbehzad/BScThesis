//
// Created by Kian Behzad on 3/13/21.
//

#include "pack_ai/ai/coach.h"

Coach::Coach()
{
    waypoints_state = 0;

    // formation control
    formation_gr1.add_vertex(rcsc::Vector2D{0, 0});
    formation_gr1.add_vertex(rcsc::Vector2D{-.5, 0});
    formation_gr1.add_vertex(rcsc::Vector2D{-.5, -.5});
    formation_gr1.add_vertex(rcsc::Vector2D{0, -.5});
    formation_gr1.add_edge(0, 1);
    formation_gr1.add_edge(0, 2);
    formation_gr1.add_edge(0, 3);
    formation_gr1.add_edge(1, 2);
    formation_gr1.add_edge(1, 3);
    formation_gr1.add_edge(2, 3);
}

Coach::~Coach() = default;

void Coach::execute()
{
    std::vector<rcsc::Vector2D> vels;
    std::vector<int> ids{0, 1, 2, 3};
    double error = formation_acquisition(ids, formation_gr1, extern_formation_acquisition_step, vels);
    for (int i{}; i < static_cast<int>(ids.size()); i++)
        extern_skill_handler->direct_velocity(ids[i], vels[i]);
}

double Coach::formation_acquisition(const std::vector<int>& robot_ids, const Graph& formation, const double& step, std::vector<rcsc::Vector2D>& vels)
{
    if (formation.vertices_num() != robot_ids.size())
    {
        qDebug() << "[ai_node] attempt to acquire a formation with different number of vertices and robots!";
        return -1;
    }

    int size = static_cast<int>(formation.vertices_num());
    vels.resize(size, rcsc::Vector2D{0, 0});

    double error{};
    // calculate robots' velocities
    for (int i{}; i < size; i++)
        for (int j{}; j < size; j++)
            if (formation.are_neighbors(i, j))
            {
                rcsc::Vector2D qtilda = extern_wm->our[ID(robot_ids[i])].pos - extern_wm->our[ID(robot_ids[j])].pos;
                double d = formation.get_dist(i, j);
                vels[i] += -step * qtilda*(qtilda.length()*qtilda.length() - d*d);
                error += fabs(qtilda.length() - d);
            }
    return error;
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

void Coach::follow_waypoints(const int& id, const QList<rcsc::Vector2D>& waypoints)
{
    int& state = waypoints_state;
    rcsc::Vector2D robot_pos = extern_wm->our[ID(id)].pos;

    double arrived_dist = 0.04;
    for (int i{}; i<waypoints.size()-1; i++)
        if (state == i && robot_pos.dist(waypoints[state]) < arrived_dist)
            state++;
    if (state == waypoints.size()-1 && robot_pos.dist(waypoints[state]) < arrived_dist)
        state = 0;

    extern_skill_handler->gotopoint_avoid(id, waypoints[state], waypoints[state]);

}



