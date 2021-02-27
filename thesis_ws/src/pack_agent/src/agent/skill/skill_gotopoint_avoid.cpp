//
// Created by Kian Behzad on 2/27/21.
//

#include "pack_agent/agent/skill/skill_gotopoint_avoid.h"

SkillGotoPointAvoid::SkillGotoPointAvoid() : Skill()
{
    angle_pid = new PID(3, 3, 4);
}

SkillGotoPointAvoid::~SkillGotoPointAvoid()
{
    delete angle_pid; angle_pid = nullptr;
}

pack_msgs::msg::RobotCommand SkillGotoPointAvoid::execute(const pack_msgs::msg::Skill& skill)
{
    // initial assignments
    skill_gotopointavoid_msg = skill.skill_gotopoint_avoid;
    id = skill.id;
    pack_msgs::msg::RobotCommand robot_command;
    robot_command.robot_id = id;

    // find the desierd robot
    pack_msgs::msg::Robot robot;
    bool found_robot = false;
    for (const auto& bot : extern_wm->our)
        if (bot.id == id)
        {   robot = bot; found_robot = true; }
    if (!found_robot)
    {   qDebug() << "[agent_node] robot id out of range (no id=" << id << " found)"; return robot_command; }


    // attraction force
    rcsc::Vector2D destination_vec = robot.pos - rcsc::Vector2D{skill_gotopointavoid_msg.destination};
    double att_rad = extern_attraction_radius;
    double att_step = extern_attraction_step;
    rcsc::Vector2D attraction = (destination_vec.length() <= att_rad) ? -att_step * destination_vec : -destination_vec.length()*att_step*destination_vec/att_rad;

    // repulsion force
    rcsc::Vector2D obstacle_center{0, 0};
    double rep_rad = extern_repulsion_radius;
    double rep_step = extern_repulsion_step;
    rcsc::Vector2D robot_obstacle_vec = (rcsc::Vector2D(robot.pos) + rcsc::Vector2D(robot.vel)*0.4) - obstacle_center;
    rcsc::Vector2D repulsion = (robot_obstacle_vec.length() <= rep_rad) ? rep_step*(1/robot_obstacle_vec.length() - 1/rep_rad)*(1/(robot_obstacle_vec.length()*robot_obstacle_vec.length()))*(robot_obstacle_vec/robot_obstacle_vec.length()) : rcsc::Vector2D{0, 0};

    rcsc::Vector2D final_force = attraction + repulsion;
    if (final_force.length() > extern_max_vel) final_force.setLength(extern_max_vel);

    rcsc::Vector2D robot_dir = rcsc::Vector2D{robot.dir}.norm();
    rcsc::Vector2D robot_norm_dir = robot_dir.rotatedVector(90);

    double velf = (final_force.x*robot_norm_dir.y - final_force.y*robot_norm_dir.x) / (robot_dir.x*robot_norm_dir.y - robot_dir.y*robot_norm_dir.x);
    double veln = (final_force.y-velf*robot_dir.y)/(robot_norm_dir.y);

    // angle PID controller
    angle_pid->set_p(extern_P_angle);
    angle_pid->set_i(extern_I_angle);
    angle_pid->set_d(extern_D_angle);

    double error_angle = rcsc::Vector2D::angleBetween_customized(robot_dir, rcsc::Vector2D(skill_gotopointavoid_msg.look_at) - robot.pos, true).degree();
    double output_angle = angle_pid->execute(error_angle);


    extern_drawer->choose_pen("darkgray", false);
    extern_drawer->draw_line(robot.pos, rcsc::Vector2D(skill_gotopointavoid_msg.destination));

    extern_drawer->choose_pen("red", true);
    extern_drawer->draw_radial_gradient(obstacle_center, rep_rad);

    extern_drawer->choose_pen("lightsalmon", false);
    extern_drawer->draw_line(robot.pos+robot_dir*0.21, robot.pos+robot_dir*(rcsc::Vector2D(robot.pos).dist(skill_gotopointavoid_msg.look_at)));



    // fill the robot command message
    robot_command.vel_f = velf;
    robot_command.vel_n = veln;
    robot_command.vel_w = output_angle;
    robot_command.wheels_speed = false;

    return robot_command;
}