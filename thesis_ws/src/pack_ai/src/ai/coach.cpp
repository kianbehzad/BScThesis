//
// Created by Kian Behzad on 3/13/21.
//

#include "pack_ai/ai/coach.h"

Coach::Coach()
{
    waypoints_state = 0;

    // formation control
    formation_gr1.add_vertex(rcsc::Vector2D{0, 0});
    formation_gr1.add_vertex(rcsc::Vector2D{-.5, 0.25});
    formation_gr1.add_vertex(rcsc::Vector2D{-.5, -.25});
    formation_gr1.add_edge(0, 1);
    formation_gr1.add_edge(0, 2);
    formation_gr1.add_edge(1, 2);

    pid_formation_rotation = new control_tool::PID(3, 3, 1);
}

Coach::~Coach() = default;

void Coach::execute()
{
    // initialize needed values
    std::vector<rcsc::Vector2D> vels;
    std::vector<int> ids{0, 1, 2};

    // calculate the translational velocity for agents in formation (vd)
    rcsc::Vector2D vd = follow_waypoints(ids[0], {{2, -2}, {2, 2}, {-2, 2}, {-2, -2}});

    // calculate the rotational velocity for agents in formation (wd)
    double wd = formation_angle_control(ids, {0, -1});

    // formation maneuvering according to vd and wd
    double formation_error = formation_maneuvering(ids, formation_gr1, {0, 0}, extern_formation_acquisition_step, vd, wd);

}

double Coach::formation_acquisition(const std::vector<int>& robot_ids,
                                    const Graph& formation,
                                    const double& step,
                                    std::vector<rcsc::Vector2D>& vels)
{
    if (formation.vertices_num() != robot_ids.size() || formation.vertices_num() == 0)
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
    return error/(formation.get_edges().size()*2.0);
}

double Coach::formation_maneuvering(const std::vector<int>& robot_ids,
                                    const Graph& formation,
                                    const rcsc::Vector2D& look_at,
                                    const double& acquisition_step,
                                    const rcsc::Vector2D& vel_d,
                                    const double& w_d) // TODO add w_d functionality
{
    // check the size
    if (formation.vertices_num() != robot_ids.size() || formation.vertices_num() == 0)
    {
        qDebug() << "[ai_node] attempt to acquire a formation with different number of vertices and robots!";
        return -1;
    }
    int size = static_cast<int>(formation.vertices_num());

    // formation acquisition
    std::vector<rcsc::Vector2D> vels(size, {0, 0});
    double error = formation_acquisition(robot_ids, formation, acquisition_step, vels);

    // formation maneuvering
    for (int i{}; i<size; i++)
    {
        // translation i.e. flocking
        vels[i] += vel_d;
        // rotation
        if (error < 0.6)
        {
            rcsc::Vector2D leader_robot = extern_wm->our[ID(robot_ids[0])].pos - extern_wm->our[ID(robot_ids[i])].pos;
            leader_robot = leader_robot.rotate(-90).setLengthVector(1);
            vels[i] += w_d*leader_robot;
        }
    }

    // apply the calculated vels to robots
    for (int i{}; i<size; i++)
        extern_skill_handler->direct_velocity(robot_ids[i], vels[i], look_at);
    return error;
}

double Coach::formation_angle_control(const std::vector<int>& robot_ids, const rcsc::Vector2D& desired_direction)
{
    if (robot_ids.empty())
    {
        qDebug() << "[ai_node] attempt to control angle if a formation with zero robots!";
        return 0;
    }
    double wd = 0;
    // calculate the middle point of the formation
    rcsc::Vector2D middle{0, 0};
    for (const auto& id : robot_ids)
        middle += extern_wm->our[ID(id)].pos;
    middle /= robot_ids.size();

    // control middle point to be on the desired_direction
    rcsc::Vector2D middole_vec = middle - extern_wm->our[ID(robot_ids[0])].pos;
    double error = rcsc::Vector2D::angleBetween_customized(middole_vec, desired_direction, true).degree();
    pid_formation_rotation->set_p(extern_P_angle);
    pid_formation_rotation->set_i(extern_I_angle);
    pid_formation_rotation->set_d(extern_D_angle);
    wd = pid_formation_rotation->execute(error);

    return wd;
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

void Coach::follow_waypoints_skill(const int& id, const QList<rcsc::Vector2D>& waypoints)
{
    int& state = waypoints_state_skill;
    rcsc::Vector2D robot_pos = extern_wm->our[ID(id)].pos;

    double arrived_dist = 0.04;
    for (int i{}; i<waypoints.size()-1; i++)
        if (state == i && robot_pos.dist(waypoints[state]) < arrived_dist)
            state++;
    if (state == waypoints.size()-1 && robot_pos.dist(waypoints[state]) < arrived_dist)
        state = 0;

    extern_skill_handler->gotopoint_avoid(id, waypoints[state], waypoints[state]);
}

rcsc::Vector2D Coach::follow_waypoints(const int& id, const QList<rcsc::Vector2D>& waypoints)
{
    int& state = waypoints_state;
    rcsc::Vector2D robot_pos = extern_wm->our[ID(id)].pos;

    double arrived_dist = 0.04;
    for (int i{}; i<waypoints.size()-1; i++)
        if (state == i && robot_pos.dist(waypoints[state]) < arrived_dist)
            state++;
    if (state == waypoints.size()-1 && robot_pos.dist(waypoints[state]) < arrived_dist)
        state = 0;

    return control_tool::calculate_attraction_classic(robot_pos, waypoints[state], extern_attraction_radius, extern_attraction_step);
}



