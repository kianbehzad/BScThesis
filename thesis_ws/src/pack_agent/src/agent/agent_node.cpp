//
// Created by Kian Behzad on 1/15/21.
//

#include "pack_agent/agent/agent_node.h"

// extern value definitions
pack_msgs::msg::WorldModel::SharedPtr extern_wm;
double extern_P_vel = 1;
double extern_I_vel = 0;
double extern_D_vel = 0;
double extern_P_pos = 1;
double extern_I_pos = 0;
double extern_D_pos = 0;
double extern_max_vel = 4;

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
    extern_P_vel = parameters_client->get_parameter("P_vel", extern_P_vel);
    extern_I_vel = parameters_client->get_parameter("I_vel", extern_I_vel);
    extern_D_vel = parameters_client->get_parameter("D_vel", extern_D_vel);
    extern_P_pos = parameters_client->get_parameter("P_pos", extern_P_pos);
    extern_I_pos = parameters_client->get_parameter("I_pos", extern_I_pos);
    extern_D_pos = parameters_client->get_parameter("D_pos", extern_D_pos);
    extern_max_vel = parameters_client->get_parameter("max_vel", extern_max_vel);

    // set up world_model callback
    worldmodel_subscription = this->create_subscription<pack_msgs::msg::WorldModel>("/world_model", 10, std::bind(&AgentNode::worldmodel_callback, this, _1));

    // set up skill callback
    skill_subscription = this->create_subscription<pack_msgs::msg::Skill>("/skill", 10, std::bind(&AgentNode::skill_callback, this, _1));

    // set up robot command publisher
    robotcommand_publisher = this->create_publisher<pack_msgs::msg::RobotCommand>("/agent_0/command", 5);

}

void AgentNode::worldmodel_callback(const pack_msgs::msg::WorldModel::SharedPtr msg)
{
    extern_wm = msg;
}

void AgentNode::skill_callback(const pack_msgs::msg::Skill::SharedPtr msg)
{
    skill = msg;
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
                extern_P_vel = changed_parameter.value.double_value;
                if(extern_P_vel == 0)  extern_P_vel = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "I_vel")
            {
                extern_I_vel = changed_parameter.value.double_value;
                if(extern_I_vel == 0)  extern_I_vel = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "D_vel")
            {
                extern_D_vel = changed_parameter.value.double_value;
                if(extern_D_vel == 0)  extern_D_vel = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "P_pos")
            {
                extern_P_pos = changed_parameter.value.double_value;
                if(extern_P_pos == 0)  extern_P_pos = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "I_pos")
            {
                extern_I_pos = changed_parameter.value.double_value;
                if(extern_I_pos == 0)  extern_I_pos = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "D_pos")
            {
                extern_D_pos = changed_parameter.value.double_value;
                if(extern_D_pos == 0)  extern_D_pos = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "max_vel")
            {
                extern_max_vel = changed_parameter.value.double_value;
                if(extern_max_vel == 0)  extern_max_vel = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
        }
        for (auto & deleted_parameter : event->deleted_parameters) {
            //do stuff
        }
    };
}
