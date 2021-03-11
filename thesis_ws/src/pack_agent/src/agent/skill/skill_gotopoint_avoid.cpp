//
// Created by Kian Behzad on 2/27/21.
//

#include "pack_agent/agent/skill/skill_gotopoint_avoid.h"

SkillGotoPointAvoid::SkillGotoPointAvoid() : Skill()
{
    angle_pid = new PID(3, 3, 4);
    local_minimum_counter = 0;
}

SkillGotoPointAvoid::~SkillGotoPointAvoid()
{
    delete angle_pid; angle_pid = nullptr;
}

pack_msgs::msg::RobotCommand SkillGotoPointAvoid::execute(const pack_msgs::msg::Robot& robot, const pack_msgs::msg::Skill& skill)
{
    // initial assignments
    skill_gotopointavoid_msg = skill.skill_gotopoint_avoid;
    pack_msgs::msg::RobotCommand robot_command;
    robot_command.robot_id = robot.id;


    // attraction force
    rcsc::Vector2D attraction = control_tool::calculate_attraction_classic(robot.pos,skill_gotopointavoid_msg.destination, extern_attraction_radius, extern_attraction_step);
    // draw
    extern_drawer->choose_pen("darkgray", false);
    extern_drawer->draw_line(robot.pos, rcsc::Vector2D(skill_gotopointavoid_msg.destination));

    // repulsion force
    // store all obstacles
    std::vector<Obstacle> obstacles;
    if (skill_gotopointavoid_msg.consider_ball_as_obstacle)
        obstacles.emplace_back(extern_wm->ball);
    if (skill_gotopointavoid_msg.consider_our_robot_as_obstacle)
        for(const auto& bot: extern_wm->our)
            if (bot.id != robot.id)
                obstacles.emplace_back(bot);
    if (skill_gotopointavoid_msg.consider_opp_robot_as_obstacle)
        for(const auto& bot: extern_wm->opp)
            obstacles.emplace_back(bot);
    if (skill_gotopointavoid_msg.consider_our_penalty_as_obstacle)
    {
        Obstacle tmp;
        tmp.obj.pos = field.ourGoal().toParsianMessage();
        tmp.repulsion_static_radius = field._PENALTY_DEPTH;
        obstacles.emplace_back(tmp);
    }
    if (skill_gotopointavoid_msg.consider_opp_penalty_as_obstacle)
    {
        Obstacle tmp;
        tmp.obj.pos = field.oppGoal().toParsianMessage();
        tmp.repulsion_static_radius = field._PENALTY_DEPTH;
        obstacles.emplace_back(tmp);
    }


    // calculate repulsion
    rcsc::Vector2D repulsion{0, 0};
    for(const auto& obs: obstacles)
    {
        //repulsion += control_tool::calculate_repulsion_classic(robot, extern_wm->ball.pos, extern_repulsion_radius, extern_repulsion_step, 0.4);
        //repulsion += control_tool::calculate_repulsion_GNRON(robot, extern_wm->ball.pos, skill_gotopointavoid_msg.destination, extern_repulsion_static_radius, extern_repulsion_static_step, 0.4, 10);
        repulsion += control_tool::calculate_repulsion_dynamic(robot, obs.obj,
                                                    skill_gotopointavoid_msg.destination,
                                                               obs.repulsion_static_radius,
                                                               obs.repulsion_static_step,
                                                               obs.repulsion_static_prediction,
                                                               obs.repulsion_dynamic_step,
                                                               obs.repulsion_dynamic_prediction,
                                                               obs.GNRON_degree);
        //draw
        extern_drawer->choose_pen("red", true);
        extern_drawer->draw_radial_gradient(obs.obj.pos, obs.repulsion_static_radius);
    }


    // final force calculation
    rcsc::Vector2D final_force = attraction + repulsion;
    if (final_force.length() > extern_max_vel) final_force.setLength(extern_max_vel);

    // solve local minimum problem
    if (rcsc::Vector2D{robot.vel}.length() < 0.7 && rcsc::Vector2D{robot.pos}.dist(skill_gotopointavoid_msg.destination) > 0.2)
        local_minimum_counter++;
    else
        local_minimum_counter = 0;
    if (local_minimum_counter > 35) // local minimum detected
    {
        rcsc::Vector2D destination_robot_dir = (rcsc::Vector2D{skill_gotopointavoid_msg.destination} - rcsc::Vector2D{robot.pos}).norm();
        destination_robot_dir.rotate(90);
        destination_robot_dir.setLength(extern_max_vel);
        final_force = destination_robot_dir;
    }

    // apply final force as velocity to robot
    // calculate robot vels to attain the desied velocity
    double velf, veln;
    control_tool::calculate_robot_linear_vels(final_force, robot.dir, velf, veln);

    // angle PID controller
    double output_angle = 0;
    if (rcsc::Vector2D{skill_gotopointavoid_msg.look_at}.isValid())
    {
        angle_pid->set_p(extern_P_angle);
        angle_pid->set_i(extern_I_angle);
        angle_pid->set_d(extern_D_angle);
        double error_angle = rcsc::Vector2D::angleBetween_customized(robot.dir, rcsc::Vector2D(skill_gotopointavoid_msg.look_at) - robot.pos, true).degree();
        output_angle = angle_pid->execute(error_angle);
        // draw
        extern_drawer->choose_pen("lightsalmon", false);
        extern_drawer->draw_line(robot.pos + robot.dir * 0.21, robot.pos + robot.dir *(rcsc::Vector2D(robot.pos).dist(skill_gotopointavoid_msg.look_at)));
    }

    // fill the robot command message
    robot_command.vel_f = velf;
    robot_command.vel_n = veln;
    robot_command.vel_w = output_angle;
    robot_command.wheels_speed = false;

    return robot_command;
}
