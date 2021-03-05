//
// Created by Kian Behzad on 1/19/21.
//

#ifndef PACK_WORLD_MODEL_SIMPLE_WORLDMODEL_NODE_H
#define PACK_WORLD_MODEL_SIMPLE_WORLDMODEL_NODE_H

#include <memory>
#include <chrono>
#include <string>
#include <functional>

#include "pack_world_model/simple_worldmodel/kalman_filter/kalman_filter.h"
#include "pack_msgs/msg/ssl_vision_detection.hpp"
#include "pack_msgs/msg/world_model.hpp"
#include "pack_msgs/msg/plot_world_model.hpp"
#include "pack_util/core/knowledge.h"

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class SimpleWorldModelNode : public rclcpp::Node
{
public:
    SimpleWorldModelNode(const rclcpp::NodeOptions &options);

private:
    std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr)> params_change_callback;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub;
    void define_params_change_callback_lambda_function();

    void vision_detection_callback(const pack_msgs::msg::SSLVisionDetection::SharedPtr msg);
    rclcpp::Subscription<pack_msgs::msg::SSLVisionDetection>::SharedPtr vision_detection_subscription;

    rclcpp::Publisher<pack_msgs::msg::WorldModel>::SharedPtr worldmodel_publisher;
    rclcpp::Publisher<pack_msgs::msg::PlotWorldModel>::SharedPtr plot_worldmodel_publisher;
    void publish_plotWM(const pack_msgs::msg::WorldModel& wm);

    KalmanFilter kalman_filter;

    // ros2 parameter values
    bool is_simulation;
    bool is_our_side_left;
    bool is_our_color_yellow;
    bool is_plotWM_on;
    
};

#endif //PACK_WORLD_MODEL_SIMPLE_WORLDMODEL_NODE_H
