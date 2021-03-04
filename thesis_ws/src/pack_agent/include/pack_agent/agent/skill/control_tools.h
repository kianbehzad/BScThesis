//
// Created by Kian Behzad on 1/16/21.
//

#ifndef PACK_AGENT_CONTROL_TOOLS_H
#define PACK_AGENT_CONTROL_TOOLS_H

#include "pack_util/geom/vector_2d.h"
#include "pack_util/geom/segment_2d.h"
#include "pack_msgs/msg/robot.hpp"

#include <vector>
#include <numeric>

namespace control_tool
{
    class PID {
    public:
        PID(const double &_max_integral_term = 4, const double &_max_derivative_term = 4, const double &_max_output = 8);

        void set_p(double _p);

        void set_i(double _i);

        void set_d(double _d);

        double execute(const double &error);

    private:
        double derivative_value;
        std::vector<double> integral_values;

        double max_integral_term;
        double max_derivative_term;
        double max_output;
        double last_error;

        double p{}, i{}, d{};
    };

    // calculates the desired robot linear vels in the order that the final velocity of the robot
    // would correspond to the vel_dir input
    void calculate_robot_linear_vels(const rcsc::Vector2D& vel_dir, const rcsc::Vector2D& robot_dir, double& vel_f, double& vel_n);

    // potential field functions
    rcsc::Vector2D calculate_attraction_classic(const rcsc::Vector2D& robot_center,
                                                const rcsc::Vector2D& destination,
                                                const double& attract_radius,
                                                const double& attract_step);

    rcsc::Vector2D calculate_repulsion_classic(const pack_msgs::msg::Robot& robot,
                                               const rcsc::Vector2D& obs_center,
                                               const double& obs_radius,
                                               const double& rep_step,
                                               const double& prediction);

    rcsc::Vector2D calculate_repulsion_GNRON(const pack_msgs::msg::Robot& robot,
                                             const rcsc::Vector2D& obs_center,
                                             const rcsc::Vector2D& goal_center,
                                             const double& obs_radius,
                                             const double& rep_step,
                                             const double& prediction,
                                             const int& n);

    rcsc::Vector2D calculate_repulsion_dynamic(const pack_msgs::msg::Robot& robot,
                                               const pack_msgs::msg::Robot& obstacle,
                                               const rcsc::Vector2D& goal_center,
                                               const double& obstacle_radius,
                                               const double& static_rep_step,
                                               const double& static_prediction,
                                               const double& dynamic_rep_step,
                                               const double& dynamic_prediction,
                                               const int& n);
}

#endif //PACK_AGENT_CONTROL_TOOLS_H
