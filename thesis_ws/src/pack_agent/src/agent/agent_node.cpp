//
// Created by Kian Behzad on 1/15/21.
//

#include "pack_agent/agent/agent_node.h"

AgentNode::AgentNode(const rclcpp::NodeOptions & options) : Node("agent_node", options)
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

    // set up parameter-change callback
    define_params_change_callback_lambda_function();
    parameter_event_sub = parameters_client->on_parameter_event(params_change_callback);
    
    // get parameter initial values
    P_vel = parameters_client->get_parameter("P_vel", P_vel);
    I_vel = parameters_client->get_parameter("I_vel", I_vel);
    D_vel = parameters_client->get_parameter("D_vel", D_vel);
    P_pos = parameters_client->get_parameter("P_pos", P_pos);
    I_pos = parameters_client->get_parameter("I_pos", I_pos);
    D_pos = parameters_client->get_parameter("D_pos", D_pos);
    max_vel = parameters_client->get_parameter("max_vel", max_vel);

    // set up world_model callback
    worldmodel_subscription = this->create_subscription<pack_msgs::msg::WorldModel>("/world_model", 10, std::bind(&AgentNode::worldmodel_callback, this, _1));



}

void AgentNode::worldmodel_callback(const pack_msgs::msg::WorldModel::SharedPtr msg)
{

}

void AgentNode::define_params_change_callback_lambda_function()
{
    params_change_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
        for (auto & new_parameter : event->new_parameters) {
            //do stuff
        }
        for (auto & changed_parameter : event->changed_parameters) {
            if(changed_parameter.name == "P_vel")
            {
                P_vel = changed_parameter.value.double_value;
                if(P_vel == 0)  P_vel = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "I_vel")
            {
                I_vel = changed_parameter.value.double_value;
                if(I_vel == 0)  I_vel = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "D_vel")
            {
                D_vel = changed_parameter.value.double_value;
                if(D_vel == 0)  D_vel = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "P_pos")
            {
                P_pos = changed_parameter.value.double_value;
                if(P_pos == 0)  P_pos = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "I_pos")
            {
                I_pos = changed_parameter.value.double_value;
                if(I_pos == 0)  I_pos = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "D_pos")
            {
                D_pos = changed_parameter.value.double_value;
                if(D_pos == 0)  D_pos = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "max_vel")
            {
                max_vel = changed_parameter.value.double_value;
                if(max_vel == 0)  max_vel = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
        }
        for (auto & deleted_parameter : event->deleted_parameters) {
            //do stuff
        }
    };
}
