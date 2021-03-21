//
// Created by Kian Behzad on 3/21/21.
//

#include "pack_util/control/potential_field.h"

namespace control_tool
{
    rcsc::Vector2D calculate_attraction_classic(const rcsc::Vector2D& robot_center,
                                                const rcsc::Vector2D& destination,
                                                const double& attract_radius,
                                                const double& attract_step)
    {
        rcsc::Vector2D destination_vec = robot_center - destination;
        return (destination_vec.length() <= attract_radius) ? -attract_step * destination_vec : -destination_vec.length()*attract_step*destination_vec/attract_radius;

    }

    rcsc::Vector2D calculate_repulsion_classic(const pack_msgs::msg::Robot& robot,
                                               const rcsc::Vector2D& obs_center,
                                               const double& obs_radius,
                                               const double& rep_step,
                                               const double& prediction)
    {
        rcsc::Vector2D robot_obstacle_vec = (rcsc::Vector2D(robot.pos) + rcsc::Vector2D(robot.vel)*prediction) - obs_center;
        return (robot_obstacle_vec.length() <= obs_radius) ? rep_step*(1/robot_obstacle_vec.length() - 1/obs_radius)*(1/(robot_obstacle_vec.length()*robot_obstacle_vec.length()))*(robot_obstacle_vec/robot_obstacle_vec.length()) : rcsc::Vector2D{0, 0};
    }

    rcsc::Vector2D calculate_repulsion_GNRON(const pack_msgs::msg::Robot& robot,
                                             const rcsc::Vector2D& obs_center,
                                             const rcsc::Vector2D& goal_center,
                                             const double& obs_radius,
                                             const double& rep_step,
                                             const double& prediction,
                                             const int& n)
    {
        rcsc::Vector2D robot_obstacle_vec = (rcsc::Vector2D(robot.pos) + rcsc::Vector2D(robot.vel)*prediction) - obs_center;
        if(robot_obstacle_vec.length() > obs_radius) return rcsc::Vector2D{0, 0};
        rcsc::Vector2D robot_goal_vec = (rcsc::Vector2D(robot.pos) + rcsc::Vector2D(robot.vel)*0) - goal_center;

        rcsc::Vector2D robot_obstacle_dir = robot_obstacle_vec.norm();
        rcsc::Vector2D robot_goal_dir = robot_goal_vec.norm();
        rcsc::Vector2D Frep1 = rep_step*(1/robot_obstacle_vec.length() - 1/obs_radius)*(pow(robot_goal_vec.length(), n)/pow(robot_obstacle_vec.length(), 2)) * robot_obstacle_dir;
        rcsc::Vector2D Frep2 = (n/2.0)*rep_step*pow((1/robot_obstacle_vec.length() - 1/obs_radius), 2)*pow(robot_goal_vec.length(), n-1) * -robot_goal_dir;

        return Frep1 + Frep2;
    }

    rcsc::Vector2D calculate_repulsion_dynamic(const pack_msgs::msg::Robot& robot,
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
}