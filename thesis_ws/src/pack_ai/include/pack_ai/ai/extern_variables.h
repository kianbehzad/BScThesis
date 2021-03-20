//
// Created by Kian Behzad on 3/5/21.
//

#ifndef PACK_AI_EXTERN_VARIABLES_H
#define PACK_AI_EXTERN_VARIABLES_H

#include "pack_msgs/msg/world_model.hpp"
#include "pack_msgs/msg/sand_box.hpp"
#include "pack_util/core/drawer.h"
#include "pack_util/core/skill_handler.h"

extern pack_msgs::msg::WorldModel::SharedPtr extern_wm;
extern Drawer* extern_drawer;
extern SkillHandler* extern_skill_handler;
extern pack_msgs::msg::SandBox* extern_sandbox_msg;
extern double extern_formation_acquisition_step;
extern double extern_temp_value1;
extern double extern_temp_value2;

#endif //PACK_AI_EXTERN_VARIABLES_H
