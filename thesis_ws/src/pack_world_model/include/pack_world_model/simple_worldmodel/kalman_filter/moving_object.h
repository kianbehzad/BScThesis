//
// Created by Kian Behzad on 1/19/21.
//

#ifndef PACK_WORLD_MODEL_MOVING_OBJECT_H
#define PACK_WORLD_MODEL_MOVING_OBJECT_H

#include <vector>
#include <numeric>
#include <iostream>
#include <QDebug>

#include "pack_util/geom/vector_2d.h"

class MovingObject
{
public:
    MovingObject();
    ~MovingObject();

    void execute(const double& dt, const rcsc::Vector2D& pos, const rcsc::Vector2D& dir = rcsc::Vector2D{}.invalidate());
    void step();
    rcsc::Vector2D get_pos();
    rcsc::Vector2D get_vel();
    rcsc::Vector2D get_acc();
    rcsc::Vector2D get_dir();
    double get_angular_vel();

private:
    // functions
    void logical_shift_left_all_vectors();
    void reset_linear_vectors();
    void reset_angular_vectors();

    // variables
    std::vector<rcsc::Vector2D> pos_trail;
    std::vector<rcsc::Vector2D> vel_trail;
    std::vector<rcsc::Vector2D> acc_trail;

    std::vector<rcsc::Vector2D> dir_trail;
    std::vector<double> angularvel_trail;

    std::vector<double> weights;

    int end;
    int size;
};

#endif //PACK_WORLD_MODEL_MOVING_OBJECT_H
