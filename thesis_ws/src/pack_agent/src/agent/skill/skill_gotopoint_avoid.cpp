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
    // draw
    extern_drawer->choose_pen("darkgray", false);
    extern_drawer->draw_line(robot.pos, rcsc::Vector2D(skill_gotopointavoid_msg.destination));

    // repulsion force
    rcsc::Vector2D repulsion{0, 0};

    if (skill_gotopointavoid_msg.consider_ball_as_obstacle)
    {
        //repulsion += calculate_repulsion_classic(robot, extern_wm->ball.pos, extern_repulsion_radius, extern_repulsion_step, 0.4);
        //repulsion += calculate_repulsion_GNRON(robot, extern_wm->ball.pos, skill_gotopointavoid_msg.destination, extern_repulsion_static_radius, extern_repulsion_static_step, 0.4, 10);
        repulsion += calculate_repulsion_dynamic(robot, extern_wm->ball, skill_gotopointavoid_msg.destination, extern_repulsion_static_radius, extern_repulsion_static_step, extern_repulsion_static_prediction, extern_repulsion_dynamic_step, extern_repulsion_dynamic_prediction, 10);
    }
    if (skill_gotopointavoid_msg.consider_our_robot_as_obstacle)
        for(const auto& bot: extern_wm->our)
        {
            if (bot.id == robot.id) continue;
            //repulsion += calculate_repulsion_classic(robot, bot.pos, extern_repulsion_radius, extern_repulsion_step,0.4);
            //repulsion += calculate_repulsion_GNRON(robot, bot.pos, skill_gotopointavoid_msg.destination, extern_repulsion_static_radius, extern_repulsion_static_step, extern_repulsion_static_prediction, 10);
            repulsion += calculate_repulsion_dynamic(robot, bot, skill_gotopointavoid_msg.destination, extern_repulsion_static_radius, extern_repulsion_static_step, extern_repulsion_static_prediction, extern_repulsion_dynamic_step, extern_repulsion_dynamic_prediction, 10);

        }
    if (skill_gotopointavoid_msg.consider_opp_robot_as_obstacle)
        for(const auto& bot: extern_wm->opp)
        {
            //repulsion += calculate_repulsion_classic(robot, bot.pos, extern_repulsion_radius, extern_repulsion_step,0.4);
            //repulsion += calculate_repulsion_GNRON(robot, bot.pos, skill_gotopointavoid_msg.destination, extern_repulsion_static_radius, extern_repulsion_static_step, extern_repulsion_static_prediction, 10);
            repulsion += calculate_repulsion_dynamic(robot, bot, skill_gotopointavoid_msg.destination, extern_repulsion_static_radius, extern_repulsion_static_step, extern_repulsion_static_prediction, extern_repulsion_dynamic_step, extern_repulsion_dynamic_prediction, 10);
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
    // draw
    extern_drawer->choose_pen("lightsalmon", false);
    extern_drawer->draw_line(robot.pos+robot_dir*0.21, robot.pos+robot_dir*(rcsc::Vector2D(robot.pos).dist(skill_gotopointavoid_msg.look_at)));


    // fill the robot command message
    robot_command.vel_f = velf;
    robot_command.vel_n = veln;
    robot_command.vel_w = output_angle;
    robot_command.wheels_speed = false;

    return robot_command;
}

rcsc::Vector2D SkillGotoPointAvoid::calculate_repulsion_classic(const pack_msgs::msg::Robot& robot,
                                                                const rcsc::Vector2D& obs_center,
                                                                const double& obs_radius,
                                                                const double& rep_step,
                                                                const double& prediction)
{
    // draw
    extern_drawer->choose_pen("red", true);
    extern_drawer->draw_radial_gradient(obs_center, obs_radius);

    rcsc::Vector2D robot_obstacle_vec = (rcsc::Vector2D(robot.pos) + rcsc::Vector2D(robot.vel)*prediction) - obs_center;
    return (robot_obstacle_vec.length() <= obs_radius) ? rep_step*(1/robot_obstacle_vec.length() - 1/obs_radius)*(1/(robot_obstacle_vec.length()*robot_obstacle_vec.length()))*(robot_obstacle_vec/robot_obstacle_vec.length()) : rcsc::Vector2D{0, 0};
}

rcsc::Vector2D SkillGotoPointAvoid::calculate_repulsion_GNRON(const pack_msgs::msg::Robot& robot,
                                                             const rcsc::Vector2D& obs_center,
                                                             const rcsc::Vector2D& goal_center,
                                                             const double& obs_radius,
                                                             const double& rep_step,
                                                             const double& prediction,
                                                             const int& n)
{
    // draw
    extern_drawer->choose_pen("red", true);
    extern_drawer->draw_radial_gradient(obs_center, obs_radius);

    rcsc::Vector2D robot_obstacle_vec = (rcsc::Vector2D(robot.pos) + rcsc::Vector2D(robot.vel)*prediction) - obs_center;
    if(robot_obstacle_vec.length() > obs_radius) return rcsc::Vector2D{0, 0};
    rcsc::Vector2D robot_goal_vec = (rcsc::Vector2D(robot.pos) + rcsc::Vector2D(robot.vel)*0) - goal_center;

    rcsc::Vector2D robot_obstacle_dir = robot_obstacle_vec.norm();
    rcsc::Vector2D robot_goal_dir = robot_goal_vec.norm();
    rcsc::Vector2D Frep1 = rep_step*(1/robot_obstacle_vec.length() - 1/obs_radius)*(pow(robot_goal_vec.length(), n)/pow(robot_obstacle_vec.length(), 2)) * robot_obstacle_dir;
    rcsc::Vector2D Frep2 = (n/2.0)*rep_step*pow((1/robot_obstacle_vec.length() - 1/obs_radius), 2)*pow(robot_goal_vec.length(), n-1) * -robot_goal_dir;

    return Frep1 + Frep2;
}

rcsc::Vector2D SkillGotoPointAvoid::calculate_repulsion_dynamic(const pack_msgs::msg::Robot& robot,
                                                                const pack_msgs::msg::Robot& obstacle,
                                                                const rcsc::Vector2D& goal_center,
                                                                const double& obstacle_radius,
                                                                const double& static_rep_step,
                                                                const double& static_prediction,
                                                                const double& dynamic_rep_step,
                                                                const double& dynamic_prediction,
                                                                const int& n)
{
    rcsc::Vector2D F_x = calculate_repulsion_GNRON(robot, obstacle.pos, goal_center, obstacle_radius, static_rep_step, static_prediction, n);
    rcsc::Vector2D F_v{0, 0};

    rcsc::Segment2D rob_prediction_seg{robot.pos+robot.vel*0.1, robot.pos+robot.vel*dynamic_prediction};
    rcsc::Segment2D obs_prediction_seg{obstacle.pos, obstacle.pos+obstacle.vel*dynamic_prediction};

    rcsc::Vector2D intersection = rob_prediction_seg.intersection(obs_prediction_seg);
    if (intersection.isValid())
    {
        double t_rob_intersect = rcsc::Vector2D{robot.pos}.dist(intersection)/ rcsc::Vector2D{obstacle.vel}.length();
        double t_obs_intersect = rcsc::Vector2D{obstacle.pos}.dist(intersection)/ rcsc::Vector2D{obstacle.vel}.length();
        double collide_probability = 1/(10*fabs(t_rob_intersect - t_obs_intersect)+1);
        rcsc::Vector2D retreat_dir = (obstacle.pos - intersection).norm();
        F_v = dynamic_rep_step*collide_probability * retreat_dir;
    }

    return F_x + F_v;
}