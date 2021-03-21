//
// Created by Kian Behzad on 3/21/21.
//

#ifndef PACK_UTIL_POTENTIAL_FIELD_H
#define PACK_UTIL_POTENTIAL_FIELD_H

#include "pack_util/geom/vector_2d.h"
#include "pack_util/geom/segment_2d.h"
#include "pack_msgs/msg/robot.hpp"

namespace control_tool
{
    rcsc::Vector2D calculate_attraction_classic(const rcsc::Vector2D& robot_center,
                                                const rcsc::Vector2D& destination,
                                                const double& attract_radius,
                                                const double& attract_step);

    rcsc::Vector2D calculate_repulsion_classic(const pack_msgs::msg::Robot& robot,
                                               const rcsc::Vector2D& obs_center,
                                               const double& obs_radius,
                                               const double& rep_step,
                                               const double& prediction);

    rcsc::Vector2D calculate_repulsion_GNRON(const pack_msgs::msg::Robot& robot,
                                             const rcsc::Vector2D& obs_center,
                                             const rcsc::Vector2D& goal_center,
                                             const double& obs_radius,
                                             const double& rep_step,
                                             const double& prediction,
                                             const int& n);

    rcsc::Vector2D calculate_repulsion_dynamic(const pack_msgs::msg::Robot& robot,
                                               const pack_msgs::msg::Robot& obstacle,
                                               const rcsc::Vector2D& goal_center,
                                               const double& obstacle_radius,
                                               const double& static_rep_step,
                                               const double& static_prediction,
                                               const double& dynamic_rep_step,
                                               const double& dynamic_prediction,
                                               const int& n);
}

#endif //PACK_UTIL_POTENTIAL_FIELD_H
