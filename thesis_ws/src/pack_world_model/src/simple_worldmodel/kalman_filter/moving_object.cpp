//
// Created by Kian Behzad on 1/19/21.
//

#include "pack_world_model/simple_worldmodel/kalman_filter/moving_object.h"

MovingObject::MovingObject()
{
    //initial assignments
    size = 10;
    end = size - 1;
    reset_linear_vectors();
    reset_angular_vectors();

    weights.resize(size, 0.0);
    for(int i{end}; i >= 0; i--)
       weights[i] = (i + 1)*(i + 1)*(i + 1);


}

MovingObject::~MovingObject()
{

}

void MovingObject::execute(const double& dt, const rcsc::Vector2D& pos, const rcsc::Vector2D& dir)
{
    // test for object replacement -> reset all vectors
    if (pos_trail[end-1].isValid())
        if(((pos - pos_trail[end-1])).length() > 0.15)
            reset_linear_vectors();

    pos_trail[end] = pos; //initial guess
    double sum = 0.0;
    rcsc::Vector2D tmp{0, 0};
    for (int i{end}; i >= 0; i--)
        if (pos_trail[i].isValid())
        {
            tmp += (pos_trail[i] + dt*(end-i)*vel_trail[i]) * weights[i];
            sum += weights[i];
        }
    pos_trail[end] = tmp/sum; //final guess


    if (pos_trail[end-1].isValid())
        vel_trail[end] = (pos_trail[end] - pos_trail[end-1]) / dt; //initial guess
    else
        vel_trail[end] = rcsc::Vector2D{0, 0}; //initial guess

    sum = 0.0;
    tmp.assign(0, 0);
    for (int i{end}; i >= 0; i--)
        if (vel_trail[i].isValid())
        {
            tmp += (vel_trail[i] + dt*(end-i)*acc_trail[i]) * weights[i];
            sum += weights[i];
        }
    vel_trail[end] = tmp/sum; //final guess


    if (vel_trail[end-1].isValid())
        acc_trail[end] = (vel_trail[end] - vel_trail[end-1]) / dt; //final guess (no filter)
    else
        acc_trail[end] = rcsc::Vector2D{0, 0}; //final guess (no filter)

    if (dir.isValid())
    {
        dir_trail[end] = dir; // initial guess
        sum = 0.0;
        tmp.assign(0, 0);
        for (int i{end}; i >= 0; i--)
            if (dir_trail[i].isValid())
            {
                tmp += (dir_trail[i].rotatedVector(dt*(end-i)*angularvel_trail[i])) * weights[i];
                sum += weights[i];
            }
        dir_trail[end] = tmp/sum; //final guess


        if (dir_trail[end-1].isValid())
        {
            rcsc::Vector2D dir1{dir_trail[end-1]};
            rcsc::Vector2D dir2{dir_trail[end]};
            double delta_theta = dir2.th().degree() - dir1.th().degree();
            int sign = delta_theta >= 0 ? +1 : -1;
            double compliment_delta_theta = delta_theta - 360*sign;
            //choose narrower angle between delta_theta and compliment_delta_theta
            double final_delta_theta = min(fabs(delta_theta), fabs(compliment_delta_theta)) == fabs(delta_theta) ? delta_theta : compliment_delta_theta;
            angularvel_trail[end] = final_delta_theta/dt; //final guess (no filter)
        }
        else
            angularvel_trail[end] = 0.0; //final guess (no filter)

    }

}

void MovingObject::step()
{
    logical_shift_left_all_vectors();
}

rcsc::Vector2D MovingObject::get_pos()
{
    return pos_trail[end];
}

rcsc::Vector2D MovingObject::get_vel()
{
    return vel_trail[end];
}

rcsc::Vector2D MovingObject::get_acc()
{
    return acc_trail[end];
}

rcsc::Vector2D MovingObject::get_dir()
{
    return dir_trail[end];
}

double MovingObject::get_angular_vel()
{
    return angularvel_trail[end];
}
void MovingObject::logical_shift_left_all_vectors()
{
    pos_trail.erase(pos_trail.begin());
    pos_trail.push_back(rcsc::Vector2D{}.invalidate());

    vel_trail.erase(vel_trail.begin());
    vel_trail.push_back(rcsc::Vector2D{}.invalidate());

    acc_trail.erase(acc_trail.begin());
    acc_trail.push_back(rcsc::Vector2D{}.invalidate());

    dir_trail.erase(dir_trail.begin());
    dir_trail.push_back(rcsc::Vector2D{}.invalidate());

    angularvel_trail.erase(angularvel_trail.begin());
    angularvel_trail.push_back(0.0);
}

void MovingObject::reset_linear_vectors()
{
    pos_trail.clear();
    vel_trail.clear();
    acc_trail.clear();
    pos_trail.resize(size, rcsc::Vector2D{}.invalidate());
    vel_trail.resize(size, rcsc::Vector2D{}.invalidate());
    acc_trail.resize(size, rcsc::Vector2D{}.invalidate());
}

void MovingObject::reset_angular_vectors()
{
    dir_trail.clear();
    angularvel_trail.clear();
    dir_trail.resize(size, rcsc::Vector2D{}.invalidate());
    angularvel_trail.resize(size, 5000.0);
}
