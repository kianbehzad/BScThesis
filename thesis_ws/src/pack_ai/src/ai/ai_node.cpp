//
// Created by Kian Behzad on 3/5/21.
//

#include "pack_ai/ai/ai_node.h"

pack_msgs::msg::WorldModel::SharedPtr extern_wm;
double extern_temp_value1 = 0;
double extern_temp_value2 = 0;

AINode::AINode(const rclcpp::NodeOptions & options) : Node("ai_node", options)
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
    extern_temp_value1 = parameters_client->get_parameter("temp_value1", extern_temp_value1);
    extern_temp_value2 = parameters_client->get_parameter("temp_value2", extern_temp_value2);

    // set up world_model callback
    worldmodel_subscription = this->create_subscription<pack_msgs::msg::WorldModel>("/world_model", 10, std::bind(&AINode::worldmodel_callback, this, _1));


}

AINode::~AINode()
{

}

void AINode::worldmodel_callback(const pack_msgs::msg::WorldModel::SharedPtr msg)
{
    extern_wm = msg;
    qDebug() << extern_wm->ball.pos.x;
}

void AINode::define_params_change_callback_lambda_function()
{
    params_change_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
        for (auto & new_parameter : event->new_parameters) {
            //do stuff
        }
        for (auto & changed_parameter : event->changed_parameters) {
            if(changed_parameter.name == "temp_value1")
            {
                extern_temp_value1 = changed_parameter.value.double_value;
                if(extern_temp_value1 == 0)  extern_temp_value1 = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "temp_value2")
            {
                extern_temp_value2 = changed_parameter.value.double_value;
                if(extern_temp_value2 == 0)  extern_temp_value2 = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
        }
        for (auto & deleted_parameter : event->deleted_parameters) {
            //do stuff
        }
    };
}
