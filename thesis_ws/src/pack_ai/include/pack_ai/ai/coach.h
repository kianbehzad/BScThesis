//
// Created by Kian Behzad on 3/13/21.
//

#ifndef PACK_AI_COACH_H
#define PACK_AI_COACH_H

#include "pack_util/core/field.h"
#include "pack_util/core/knowledge.h"
#include "pack_util/geom/segment_2d.h"
#include "pack_util/geom/rect_2d.h"
#include "pack_util/control/potential_field.h"
#include "pack_ai/ai/extern_variables.h"
#include "pack_ai/ai/tools/graph.h"

#include <QDebug>

class Coach
{
public:
    Coach();
    ~Coach();
    void execute();

private:
    CField field;

    // gets the desired id and returns the index corresponding to the id in wm.our
    static int ID(int id);

    // push ball to a destination
    void push_ball(const int& id, const rcsc::Vector2D& destination);

    // defense at penalty area
    void defense_at_penalty(const int& id);

    // follow a set of waypoints continuously by gotopoint_avoid skill
    void follow_waypoints_skill(const int& id, const QList<rcsc::Vector2D>& waypoints);
    int waypoints_state_skill;

    // follow a set of waypoints continuously by returning the necessary velocity of the robot
    rcsc::Vector2D follow_waypoints(const int& id, const QList<rcsc::Vector2D>& waypoints);
    int waypoints_state;

    // formation control

    // this function gets a formation graph and the desired robot IDs then computes the desired velocities to form the formation
    // desired velocities are stored in the vels input. the function returns sum of the formation errors
    double formation_acquisition(const std::vector<int>& robot_ids,
                                 const Graph& formation,
                                 const double& step,
                                 std::vector<rcsc::Vector2D>& vels);

    // this function uses formation_acquisition to form a formation with robots and then moves it with:
    // vel_d - is the translation velocity of the formation
    // w_d - is the rotational velocity of the formation with vertex 0 as rotational axes
    // the function returns sum of the formation errors
    double formation_maneuvering(const std::vector<int>& robot_ids,
                                 const Graph& formation,
                                 const rcsc::Vector2D& look_at,
                                 const double& acquisition_step,
                                 const rcsc::Vector2D& vel_d,
                                 const double& w_d);
    Graph formation_gr1;

};

#endif //PACK_AI_COACH_H
