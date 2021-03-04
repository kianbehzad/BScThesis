//
// Created by Kian Behzad on 1/16/21.
//

#include "pack_agent/agent/skill/control_tools.h"

namespace control_tool
{
    PID::PID(const double &_max_integral_term, const double &_max_derivative_term, const double &_max_output)
            : max_integral_term{_max_integral_term}, max_derivative_term{_max_derivative_term},
              max_output{_max_output} {
        p = i = d = 0;
        last_error = 0;
        integral_values = std::vector<double>(10, 0);
    }

    double PID::execute(const double &error) {
        integral_values.erase(integral_values.begin());
        integral_values.push_back(error);

        double integral_term = i * accumulate(integral_values.begin(), integral_values.end(), 0.0);
        integral_term = (integral_term > max_integral_term) ? max_integral_term : integral_term;

        double derivative_term = d * (error - last_error);
        derivative_term = (derivative_term > max_derivative_term) ? max_derivative_term : derivative_term;

        last_error = error;

        double ret = p * error + integral_term + derivative_term;
        return ret > max_output ? max_output : ret;

    }

    void PID::set_p(double _p) {
        p = _p;
    }

    void PID::set_i(double _i) {
        i = _i;
    }

    void PID::set_d(double _d) {
        d = _d;
    }


    void calculate_robot_linear_vels(const rcsc::Vector2D& vel_dir, const rcsc::Vector2D& robot_dir, double& vel_f, double& vel_n)
    {
        rcsc::Vector2D robot_norm_dir = robot_dir.rotatedVector(90);

        vel_f = (vel_dir.x*robot_norm_dir.y - vel_dir.y*robot_norm_dir.x) / (robot_dir.x*robot_norm_dir.y - robot_dir.y*robot_norm_dir.x);
        vel_n = (vel_dir.y-vel_f*robot_dir.y)/(robot_norm_dir.y);
    }

    // potential field functions
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