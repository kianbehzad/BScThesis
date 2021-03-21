//
// Created by Kian Behzad on 3/21/21.
//

#ifndef PACK_UTIL_PID_H
#define PACK_UTIL_PID_H

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
}

#endif //PACK_UTIL_PID_H