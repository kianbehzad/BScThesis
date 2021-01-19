//
// Created by Kian Behzad on 1/19/21.
//

#include "pack_world_model/simple_worldmodel/simple_worldmodel_node.h"

SimpleWorldModelNode::SimpleWorldModelNode(const rclcpp::NodeOptions &options) : Node("worldmodel_node", options)
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

    //get initial parameter values
    is_simulation = parameters_client->get_parameter("is_simulation", is_simulation);
    is_our_side_left = parameters_client->get_parameter("is_our_side_left", is_our_side_left);
    is_our_color_yellow = parameters_client->get_parameter("is_our_color_yellow", is_our_color_yellow);
    is_plotWM_on = parameters_client->get_parameter("is_plotWM_on", is_plotWM_on);

    //set up world-model publisher
    worldmodel_publisher = this->create_publisher<pack_msgs::msg::WorldModel>("/world_model", 5);
    //set up plot-world-model publisher
    plot_worldmodel_publisher = this->create_publisher<pack_msgs::msg::PlotWorldModel>("/plot_world_model", 5);

    // set up vision_detection callback
    vision_detection_subscription = this->create_subscription<pack_msgs::msg::SSLVisionDetection>("/vision_detection", 8, std::bind(&SimpleWorldModelNode::vision_detection_callback, this, _1));

}

void SimpleWorldModelNode::vision_detection_callback(const pack_msgs::msg::SSLVisionDetection::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "simple_worldmodel callback");
}

void SimpleWorldModelNode::define_params_change_callback_lambda_function()
{
    params_change_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
        for (auto & new_parameter : event->new_parameters) {
            //do stuff
        }
        for (auto & changed_parameter : event->changed_parameters) {
            if(changed_parameter.name == "is_simulation")
            {
                is_simulation = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "is_our_side_left")
            {
                is_our_side_left = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "is_our_color_yellow")
            {
                is_our_color_yellow = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "is_plotWM_on")
            {
                is_plotWM_on = changed_parameter.value.bool_value;
            }
        }
        for (auto & deleted_parameter : event->deleted_parameters) {
            //do stuff
        }
    };

}