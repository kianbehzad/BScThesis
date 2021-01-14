#include "pack_protobuf_wrapper/grsim/grsim_node.h"

GrsimNode::GrsimNode(const rclcpp::NodeOptions & options) : Node("grsim_node", options)
{

    // set up parameter client
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!parameters_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the parameter service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "parameter service not available, waiting again...");
    }

    // set up worldmodel parameter client
    auto parameters_worldmodel_client = std::make_shared<rclcpp::SyncParametersClient>(this, "worldmodel_node");
    while (!parameters_worldmodel_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the worldmodel parameter service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "worldmodel parameter service not available, waiting again...");
    }

    // initialize worldmodel parameters
    is_our_color_yellow = parameters_worldmodel_client->get_parameter("is_our_color_yellow", is_our_color_yellow);

    // set up udp connection
    grsim_ip = parameters_client->get_parameter("grsim_ip", grsim_ip);
    grsim_command_listen_port = parameters_client->get_parameter("grsim_command_listen_port", grsim_command_listen_port);
    RCLCPP_INFO(this->get_logger(), "stablish udp com: " + grsim_ip +":%d", grsim_command_listen_port);
    udp_send = new UDPSend(grsim_ip, grsim_command_listen_port);


    // set up parameter-change callback
    define_params_change_callback_lambda_function();
    parameter_event_sub = parameters_client->on_parameter_event(params_change_callback);

    // set up world_model callback
    worldmodel_subscription = this->create_subscription<pack_msgs::msg::WorldModel>("/world_model", 10, std::bind(&GrsimNode::worldmodel_callback, this, _1));

    // set up agent_command callbacks
    for(int i{}; i < knowledge::MAX_ROBOT_NUM; i++)
        command_subscription[i] = this->create_subscription<pack_msgs::msg::RobotCommand>("/agent_" + std::to_string(i) + "/command", 10, std::bind(&GrsimNode::command_callback, this, _1));

    // set up robot and ball replacement service
    ball_replacement_service = this->create_service<pack_msgs::srv::GrsimBallReplacement>("/grsim_ball_replacement",  std::bind(&GrsimNode::ball_replacement_callback, this, _1, _2));
    robot_replacement_service = this->create_service<pack_msgs::srv::GrsimRobotReplacement>("/grsim_robot_replacement",  std::bind(&GrsimNode::robot_replacement_callback, this, _1, _2));

}

void GrsimNode::command_callback(const pack_msgs::msg::RobotCommand::SharedPtr msg)
{
    grSim_Robot_Command* grsim_robot_command = grsim_commands.add_robot_commands();
    grsim_robot_command->set_id(msg->robot_id);
    grsim_robot_command->set_kickspeedx(msg->kick_speed / 100.0);
    grsim_robot_command->set_kickspeedz(msg->kick_speedz / 200.0);
    grsim_robot_command->set_veltangent(msg->vel_f);
    grsim_robot_command->set_velnormal(msg->vel_n);
    grsim_robot_command->set_velangular(msg->vel_w);
    grsim_robot_command->set_wheel1(msg->wheel1);
    grsim_robot_command->set_wheel2(msg->wheel2);
    grsim_robot_command->set_wheel3(msg->wheel3);
    grsim_robot_command->set_wheel4(msg->wheel4);
    grsim_robot_command->set_spinner(msg->spinner != 0u);
    grsim_robot_command->set_wheelsspeed(msg->wheels_speed != 0u);
}

void GrsimNode::worldmodel_callback(const pack_msgs::msg::WorldModel::SharedPtr msg)
{
    grsim_commands.set_isteamyellow(static_cast<int>(is_our_color_yellow));
    grsim_commands.set_timestamp(0.0);

    grSim_Commands* temp_grsim_commands = grsim_packet.mutable_commands();
    temp_grsim_commands->CopyFrom(grsim_commands);


    std::string buffer;
    grsim_packet.SerializeToString(&buffer);
    udp_send->send(buffer);

    grsim_packet.clear_commands();
    grsim_commands.clear_robot_commands();
}

void GrsimNode::ball_replacement_callback(const std::shared_ptr<pack_msgs::srv::GrsimBallReplacement::Request> request, std::shared_ptr<pack_msgs::srv::GrsimBallReplacement::Response> response)
{
    grSim_BallReplacement grsim_ball_replacement;
    grsim_ball_replacement.set_x(request->x);
    grsim_ball_replacement.set_y(request->y);
    grsim_ball_replacement.set_vx(request->vx);
    grsim_ball_replacement.set_vy(request->vy);

    grSim_Replacement grsim_replacement;
    grSim_BallReplacement* temp_grsim_ball_replacement = grsim_replacement.mutable_ball();
    temp_grsim_ball_replacement->CopyFrom(grsim_ball_replacement);

    grSim_Packet packet;
    grSim_Replacement* temp_grsim_replacement = packet.mutable_replacement();
    temp_grsim_replacement->CopyFrom(grsim_replacement);

    std::string buffer;
    packet.SerializeToString(&buffer);
    udp_send->send(buffer);

}

void GrsimNode::robot_replacement_callback(const std::shared_ptr<pack_msgs::srv::GrsimRobotReplacement::Request> request, std::shared_ptr<pack_msgs::srv::GrsimRobotReplacement::Response> response)
{
    grSim_Replacement grsim_replacement;
    grSim_RobotReplacement* grsim_robot_replacement = grsim_replacement.add_robots();
    grsim_robot_replacement->set_x(request->x);
    grsim_robot_replacement->set_y(request->y);
    grsim_robot_replacement->set_dir(request->dir);
    grsim_robot_replacement->set_id(request->id);
    grsim_robot_replacement->set_yellowteam(request->yellow_team);

    grSim_Packet packet;
    grSim_Replacement* temp_grsim_replacement = packet.mutable_replacement();
    temp_grsim_replacement->CopyFrom(grsim_replacement);

    std::string buffer;
    packet.SerializeToString(&buffer);
    udp_send->send(buffer);
}


void GrsimNode::define_params_change_callback_lambda_function()
{
    params_change_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
        for (auto & new_parameter : event->new_parameters) {
            //do stuff
        }
        for (auto & changed_parameter : event->changed_parameters) {
            if(changed_parameter.name == "grsim_ip")
            {
                grsim_ip = changed_parameter.value.string_value;
                udp_send->setIP(grsim_ip);
                RCLCPP_INFO(this->get_logger(), "stablish udp com: " + grsim_ip +":%d", grsim_command_listen_port);
            }
            else if(changed_parameter.name == "grsim_command_listen_port")
            {
                grsim_command_listen_port = changed_parameter.value.integer_value;
                udp_send->setport(grsim_command_listen_port);
                RCLCPP_INFO(this->get_logger(), "stablish udp com: " + grsim_ip +":%d", grsim_command_listen_port);
            }
            else if(changed_parameter.name == "is_our_color_yellow")
            {
                is_our_color_yellow = changed_parameter.value.bool_value;
                RCLCPP_INFO(this->get_logger(), "changing color, is_our_color_yellow: %d", is_our_color_yellow);
            }
        }
        for (auto & deleted_parameter : event->deleted_parameters) {
            //do stuff
        }
    };

}
