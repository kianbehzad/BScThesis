//
// Created by Kian Behzad on 1/16/21.
//

#ifndef PACK_AGENT_EXTERN_VARIABLES_H
#define PACK_AGENT_EXTERN_VARIABLES_H

#include "pack_msgs/msg/world_model.hpp"
#include "pack_util/core/drawer.h"

extern pack_msgs::msg::WorldModel::SharedPtr extern_wm;
extern Drawer* extern_drawer;
extern double extern_P_pos;
extern double extern_I_pos;
extern double extern_D_pos;
extern double extern_P_angle;
extern double extern_I_angle;
extern double extern_D_angle;
extern double extern_max_vel;
extern double extern_temp_value1;
extern double extern_temp_value2;

#endif //PACK_AGENT_EXTERN_VARIABLES_H
