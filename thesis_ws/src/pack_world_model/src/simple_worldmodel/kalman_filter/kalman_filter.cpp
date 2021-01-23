//
// Created by Kian Behzad on 1/19/21.
//

#include "pack_world_model/simple_worldmodel/kalman_filter/kalman_filter.h"

KalmanFilter::KalmanFilter()
{
    blue_objects.resize(knowledge::MAX_ROBOT_NUM);
    yellow_objects.resize(knowledge::MAX_ROBOT_NUM);
}

KalmanFilter::~KalmanFilter()
{

}

pack_msgs::msg::WorldModel KalmanFilter::execute(pack_msgs::msg::SSLVisionDetection::SharedPtr detection, const bool& is_yellow, const bool& is_left)
{
    // step all moving abjects
    for(auto& bot : blue_objects)
        bot.step();
    for(auto& bot : yellow_objects)
        bot.step();
    ball_object.step();

    // execute insight objects
    double dt = 1.0/60.0; //double dt = detection->t_capture - last_time; --> 1/60 works way better
    for(const auto& blue : detection->blue)
        blue_objects[blue.robot_id].execute(dt, blue.pos, rcsc::Vector2D{}.setPolar(1, rcsc::AngleDeg::rad2deg(blue.orientation)));

    for(const auto& yellow : detection->yellow)
        yellow_objects[yellow.robot_id].execute(dt, yellow.pos, rcsc::Vector2D{}.setPolar(1, rcsc::AngleDeg::rad2deg(yellow.orientation)));

    ball_object.execute(dt, detection->balls[0].pos);


    // save time for next loop
    last_time = detection->t_capture;


    // fill world model message
    pack_msgs::msg::WorldModel wm;
    float coef = is_left ? +1.0 : -1.0;

    for(const auto& blue : detection->blue)
    {
        pack_msgs::msg::Robot robot;
        robot.pos = (coef*blue_objects[blue.robot_id].get_pos()).toParsianMessage();
        robot.vel = (coef*blue_objects[blue.robot_id].get_vel()).toParsianMessage();
        robot.acc = (coef*blue_objects[blue.robot_id].get_acc()).toParsianMessage();
        robot.dir = (coef*blue_objects[blue.robot_id].get_dir()).toParsianMessage();
        robot.angular_vel = blue_objects[blue.robot_id].get_angular_vel();
        robot.id = blue.robot_id;
        if (!is_yellow) wm.our.push_back(robot);
        else            wm.opp.push_back(robot);
    }

    for(const auto& yellow : detection->yellow)
    {
        pack_msgs::msg::Robot robot;
        robot.pos = (coef*yellow_objects[yellow.robot_id].get_pos()).toParsianMessage();
        robot.vel = (coef*yellow_objects[yellow.robot_id].get_vel()).toParsianMessage();
        robot.acc = (coef*yellow_objects[yellow.robot_id].get_acc()).toParsianMessage();
        robot.dir = (coef*yellow_objects[yellow.robot_id].get_dir()).toParsianMessage();
        robot.angular_vel = yellow_objects[yellow.robot_id].get_angular_vel();
        robot.id = yellow.robot_id;
        if (!is_yellow) wm.opp.push_back(robot);
        else            wm.our.push_back(robot);
    }

    wm.ball.pos = (coef*ball_object.get_pos()).toParsianMessage();
    wm.ball.vel = (coef*ball_object.get_vel()).toParsianMessage();
    wm.ball.acc = (coef*ball_object.get_acc()).toParsianMessage();

    return wm;
}