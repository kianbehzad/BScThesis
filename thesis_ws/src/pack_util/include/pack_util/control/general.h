//
// Created by Kian Behzad on 3/21/21.
//

#ifndef PACK_UTIL_GENERAL_H
#define PACK_UTIL_GENERAL_H

#include "pack_util/geom/vector_2d.h"

namespace control_tool
{

    // calculates the desired robot linear vels in the order that the final velocity of the robot
    // would correspond to the vel_dir input
    void calculate_robot_linear_vels(const rcsc::Vector2D &vel_dir,
                                     const rcsc::Vector2D &robot_dir,
                                     double &vel_f,
                                     double &vel_n);
}

#endif //PACK_UTIL_GENERAL_H
