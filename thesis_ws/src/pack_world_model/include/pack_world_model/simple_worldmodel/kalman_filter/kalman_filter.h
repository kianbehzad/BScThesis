//
// Created by Kian Behzad on 1/19/21.
//

#ifndef PACK_WORLD_MODEL_KALMAN_FILTER_H
#define PACK_WORLD_MODEL_KALMAN_FILTER_H

#include <vector>
#include <QDebug>

#include "pack_world_model/simple_worldmodel/kalman_filter/moving_object.h"
#include "pack_util/core/knowledge.h"
#include "pack_msgs/msg/world_model.hpp"
#include "pack_msgs/msg/ssl_vision_detection.hpp"

class KalmanFilter
{
public:
    KalmanFilter();
    ~KalmanFilter();

    pack_msgs::msg::WorldModel execute(pack_msgs::msg::SSLVisionDetection::SharedPtr detection, const bool& is_yellow, const bool& is_left);

private:
    std::vector<MovingObject> blue_objects;
    std::vector<MovingObject> yellow_objects;
    MovingObject ball_object;

    double last_time;
};

#endif //PACK_WORLD_MODEL_KALMAN_FILTER_H
