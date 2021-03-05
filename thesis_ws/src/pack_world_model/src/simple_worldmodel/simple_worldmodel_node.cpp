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
    plot_worldmodel_publisher = this->create_publisher<pack_msgs::msg::PlotWorldModel>("/plotter", 5);

    // set up vision_detection callback
    vision_detection_subscription = this->create_subscription<pack_msgs::msg::SSLVisionDetection>("/vision_detection", 8, std::bind(&SimpleWorldModelNode::vision_detection_callback, this, _1));

}

void SimpleWorldModelNode::vision_detection_callback(const pack_msgs::msg::SSLVisionDetection::SharedPtr msg)
{
    pack_msgs::msg::WorldModel wm = kalman_filter.execute(msg, is_our_color_yellow, is_our_side_left);
    wm.is_left = is_our_side_left;
    wm.is_yellow = is_our_color_yellow;
    wm.header.stamp = rclcpp::Node::now();

    worldmodel_publisher->publish(wm);
    if (is_plotWM_on) publish_plotWM(wm);
}

void SimpleWorldModelNode::publish_plotWM(const pack_msgs::msg::WorldModel& wm)
{
    pack_msgs::msg::PlotWorldModel tmp;
    tmp.header = wm.header;
    tmp.is_yellow = wm.is_yellow;
    tmp.is_left = wm.is_left;
    tmp.ball = wm.ball;

    if (wm.our.size()>0)   tmp.our0 = wm.our[0];
    if (wm.our.size()>1)   tmp.our1 = wm.our[1];
    if (wm.our.size()>2)   tmp.our2 = wm.our[2];
    if (wm.our.size()>3)   tmp.our3 = wm.our[3];
    if (wm.our.size()>4)   tmp.our4 = wm.our[4];
    if (wm.our.size()>5)   tmp.our5 = wm.our[5];
    if (wm.our.size()>6)   tmp.our6 = wm.our[6];
    if (wm.our.size()>7)   tmp.our7 = wm.our[7];
    if (wm.our.size()>8)   tmp.our8 = wm.our[8];
    if (wm.our.size()>9)   tmp.our9 = wm.our[9];
    if (wm.our.size()>10)  tmp.our10 = wm.our[10];

    if (wm.opp.size()>0)   tmp.opp0 = wm.opp[0];
    if (wm.opp.size()>1)   tmp.opp1 = wm.opp[1];
    if (wm.opp.size()>2)   tmp.opp2 = wm.opp[2];
    if (wm.opp.size()>3)   tmp.opp3 = wm.opp[3];
    if (wm.opp.size()>4)   tmp.opp4 = wm.opp[4];
    if (wm.opp.size()>5)   tmp.opp5 = wm.opp[5];
    if (wm.opp.size()>6)   tmp.opp6 = wm.opp[6];
    if (wm.opp.size()>7)   tmp.opp7 = wm.opp[7];
    if (wm.opp.size()>8)   tmp.opp8 = wm.opp[8];
    if (wm.opp.size()>9)   tmp.opp9 = wm.opp[9];
    if (wm.opp.size()>10)  tmp.opp10 = wm.opp[10];

    plot_worldmodel_publisher->publish(tmp);
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