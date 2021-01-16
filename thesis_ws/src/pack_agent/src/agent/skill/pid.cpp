//
// Created by Kian Behzad on 1/16/21.
//

#include "pack_agent/agent/skill/pid.h"

PID::PID(const double& _max_integral_term, const double& _max_derivative_term, const double& _max_output)
    : max_integral_term{_max_integral_term}, max_derivative_term{_max_derivative_term}, max_output{_max_output}
{
    p = i = d = 0;
    last_error = 0;
    integral_values = std::vector<double>(100, 0);
}

double PID::execute(const double& error)
{
    integral_values.erase(integral_values.begin());
    integral_values.push_back(error);

    double integral_term = i * accumulate(integral_values.begin(),integral_values.end(),0.0);
    integral_term = (integral_term > max_integral_term) ? max_integral_term : integral_term;

    double derivative_term = d * (error - last_error);
    derivative_term = (derivative_term > max_derivative_term) ? max_derivative_term : derivative_term;

    last_error = error;

    double ret = p*error + integral_term + derivative_term;
    return ret > max_output ? max_output : ret;

}

void PID::set_p(double _p)
{
    p = _p;
}
void PID::set_i(double _i)
{
    i = _i;
}
void PID::set_d(double _d)
{
    d = _d;
}