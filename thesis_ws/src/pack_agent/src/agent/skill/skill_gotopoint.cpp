//
// Created by Kian Behzad on 1/16/21.
//

#include "pack_agent/agent/skill/skill_gotopoint.h"

SkillGotoPoint::SkillGotoPoint() : Skill()
{
    pos_pid = new PID(3, 3, 4);
    angle_pid = new PID(3, 3, 4);
}

SkillGotoPoint::~SkillGotoPoint()
{
    delete pos_pid; pos_pid = nullptr;
}

pack_msgs::msg::RobotCommand SkillGotoPoint::execute(const pack_msgs::msg::Skill& skill)
{
    // initial assignments
    skill_gotopoint_msg = skill.skill_gotopoint;
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

    // position PID controller
    pos_pid->set_p(extern_P_pos);
    pos_pid->set_i(extern_I_pos);
    pos_pid->set_d(extern_D_pos);

    double error_pos = rcsc::Vector2D(skill_gotopoint_msg.destination).dist(rcsc::Vector2D(robot.pos));
    double output_pos = pos_pid->execute(error_pos);
    rcsc::Vector2D dir = output_pos * (rcsc::Vector2D(skill_gotopoint_msg.destination) - rcsc::Vector2D(robot.pos)).norm();

    rcsc::Vector2D robot_dir = rcsc::Vector2D{robot.dir}.norm();
    rcsc::Vector2D robot_norm_dir = robot_dir.rotatedVector(90);

    double velf = (dir.x*robot_norm_dir.y - dir.y*robot_norm_dir.x) / (robot_dir.x*robot_norm_dir.y - robot_dir.y*robot_norm_dir.x);
    double veln = (dir.y-velf*robot_dir.y)/(robot_norm_dir.y);

    extern_drawer->choose_pen("darkgray", false);
    extern_drawer->draw_line(robot.pos, rcsc::Vector2D(skill_gotopoint_msg.destination));

    // angle PID controller
    angle_pid->set_p(extern_P_angle);
    angle_pid->set_i(extern_I_angle);
    angle_pid->set_d(extern_D_angle);

    double error_angle = rcsc::Vector2D::angleBetween_customized(robot_dir, rcsc::Vector2D(skill_gotopoint_msg.look_at) - robot.pos, true).degree();
    double output_angle = angle_pid->execute(error_angle);

    extern_drawer->choose_pen("lightsalmon", false);
    extern_drawer->draw_line(robot.pos+robot_dir*0.21, robot.pos+robot_dir*(rcsc::Vector2D(robot.pos).dist(skill_gotopoint_msg.look_at)));


    // fill the robot command message
    robot_command.vel_f = velf;
    robot_command.vel_n = veln;
    robot_command.vel_w = output_angle;
    robot_command.wheels_speed = false;

    return robot_command;

}