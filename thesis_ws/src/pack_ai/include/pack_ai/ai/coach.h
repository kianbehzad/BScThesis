//
// Created by Kian Behzad on 3/13/21.
//

#ifndef PACK_AI_COACH_H
#define PACK_AI_COACH_H

#include "pack_util/core/field.h"
#include "pack_util/core/knowledge.h"
#include "pack_util/geom/segment_2d.h"
#include "pack_util/geom/rect_2d.h"
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
    int ID(int id);

    // push ball to a destination
    void push_ball(const int& id, const rcsc::Vector2D& destination);

    // defense at penalty area
    void defense_at_penalty(const int& id);

    // follow a set of waypoints continuously
    void follow_waypoints(const int& id, const QList<rcsc::Vector2D>& waypoints);
    int waypoints_state;



};

#endif //PACK_AI_COACH_H
