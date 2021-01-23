//
// Created by kian behzad on 3/14/20.
//

#ifndef PACK_PROTOBUF_WRAPPER_GRSIM_NODE_H
#define PACK_PROTOBUF_WRAPPER_GRSIM_NODE_H

#include <memory>
#include <chrono>
#include <string>
#include <functional>

#include "pack_msgs/msg/robot_command.hpp"
#include "pack_msgs/msg/world_model.hpp"
#include "pack_msgs/srv/grsim_ball_replacement.hpp"
#include "pack_msgs/srv/grsim_robot_replacement.hpp"
#include "pack_util/core/knowledge.h"
#include "pack_protobuf_wrapper/common/net/udpsend.h"
#include "pack_protobuf_wrapper/proto/grSim_Commands.pb.h"
#include "pack_protobuf_wrapper/proto/grSim_Replacement.pb.h"
#include "pack_protobuf_wrapper/proto/grSim_Packet.pb.h"

#include "rclcpp/rclcpp.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;


class GrsimNode : public rclcpp::Node
{
public:
    GrsimNode(const rclcpp::NodeOptions & options);

private:
    std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr)> params_change_callback;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub;
    void define_params_change_callback_lambda_function();

    void worldmodel_callback(const pack_msgs::msg::WorldModel::SharedPtr msg);
    rclcpp::Subscription<pack_msgs::msg::WorldModel>::SharedPtr worldmodel_subscription;

    void command_callback(const pack_msgs::msg::RobotCommand::SharedPtr msg);
    rclcpp::Subscription<pack_msgs::msg::RobotCommand>::SharedPtr command_subscription[knowledge::MAX_ROBOT_NUM];

    void ball_replacement_callback(const std::shared_ptr<pack_msgs::srv::GrsimBallReplacement::Request> request, std::shared_ptr<pack_msgs::srv::GrsimBallReplacement::Response> response);
    rclcpp::Service<pack_msgs::srv::GrsimBallReplacement>::SharedPtr ball_replacement_service;

    void robot_replacement_callback(const std::shared_ptr<pack_msgs::srv::GrsimRobotReplacement::Request> request, std::shared_ptr<pack_msgs::srv::GrsimRobotReplacement::Response> response);
    rclcpp::Service<pack_msgs::srv::GrsimRobotReplacement>::SharedPtr robot_replacement_service;

    UDPSend* udp_send;
    std::string grsim_ip;
    int grsim_command_listen_port;

    grSim_Commands grsim_commands;
    grSim_Packet grsim_packet;


};



#endif //PACK_PROTOBUF_WRAPPER_GRSIM_NODE_H
