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
    delete angle_pid; angle_pid = nullptr;
}

pack_msgs::msg::RobotCommand SkillGotoPoint::execute(const pack_msgs::msg::Robot& robot, const pack_msgs::msg::Skill& skill)
{
    // initial assignments
    skill_gotopoint_msg = skill.skill_gotopoint;
    pack_msgs::msg::RobotCommand robot_command;
    robot_command.robot_id = robot.id;

    // position PID controller
    pos_pid->set_p(extern_P_pos);
    pos_pid->set_i(extern_I_pos);
    pos_pid->set_d(extern_D_pos);

    double error_pos = rcsc::Vector2D(skill_gotopoint_msg.destination).dist(rcsc::Vector2D(robot.pos));
    double output_pos = pos_pid->execute(error_pos);
    rcsc::Vector2D dir = output_pos * (rcsc::Vector2D(skill_gotopoint_msg.destination) - rcsc::Vector2D(robot.pos)).norm();

    // calculate robot vels to attain the desied velocity
    double velf, veln;
    control_tool::calculate_robot_linear_vels(dir, robot.dir, velf, veln);

    extern_drawer->choose_pen("darkgray", false);
    extern_drawer->draw_line(robot.pos, rcsc::Vector2D(skill_gotopoint_msg.destination));

    // angle PID controller
    double output_angle = 0;
    if (rcsc::Vector2D{skill_gotopoint_msg.look_at}.isValid())
    {
        angle_pid->set_p(extern_P_angle);
        angle_pid->set_i(extern_I_angle);
        angle_pid->set_d(extern_D_angle);

        double error_angle = rcsc::Vector2D::angleBetween_customized(robot.dir,rcsc::Vector2D(skill_gotopoint_msg.look_at) -robot.pos, true).degree();
        output_angle = angle_pid->execute(error_angle);

        extern_drawer->choose_pen("lightsalmon", false);
        extern_drawer->draw_line(robot.pos + robot.dir * 0.21,robot.pos + robot.dir * (rcsc::Vector2D(robot.pos).dist(skill_gotopoint_msg.look_at)));
    }

    // fill the robot command message
    robot_command.vel_f = velf;
    robot_command.vel_n = veln;
    robot_command.vel_w = output_angle;
    robot_command.wheels_speed = false;

    return robot_command;

}