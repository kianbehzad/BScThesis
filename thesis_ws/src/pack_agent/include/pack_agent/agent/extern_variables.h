//
// Created by Kian Behzad on 1/16/21.
//

#ifndef PACK_AGENT_EXTERN_VARIABLES_H
#define PACK_AGENT_EXTERN_VARIABLES_H

#include "pack_msgs/msg/world_model.hpp"

extern pack_msgs::msg::WorldModel::SharedPtr extern_wm;
extern double extern_I_vel;
extern double extern_D_vel;
extern double extern_P_pos;
extern double extern_I_pos;
extern double extern_D_pos;
extern double extern_max_vel;

#endif //PACK_AGENT_EXTERN_VARIABLES_H
