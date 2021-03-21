//
// Created by Kian Behzad on 3/21/21.
//

#include "pack_util/control/general.h"

namespace control_tool
{
    void calculate_robot_linear_vels(const rcsc::Vector2D &vel_dir,
                                     const rcsc::Vector2D &robot_dir,
                                     double &vel_f,
                                     double &vel_n)
    {
        rcsc::Vector2D robot_norm_dir = robot_dir.rotatedVector(90);

        vel_f = (vel_dir.x*robot_norm_dir.y - vel_dir.y*robot_norm_dir.x) / (robot_dir.x*robot_norm_dir.y - robot_dir.y*robot_norm_dir.x);
        vel_n = (vel_dir.y-vel_f*robot_dir.y)/(robot_norm_dir.y);
    }
}