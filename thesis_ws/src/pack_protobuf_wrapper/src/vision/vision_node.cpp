//
// Created by kian behzad on 3/22/20.
//

#include "pack_protobuf_wrapper/vision/vision_node.h"


VisionNode::VisionNode(const rclcpp::NodeOptions & options) : Node("vision_node", options)
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

    // set up udp connection
    vision = nullptr;
    vision_multicast_ip = parameters_client->get_parameter("vision_multicast_ip", vision_multicast_ip);
    vision_multicast_port = parameters_client->get_parameter("vision_multicast_port", vision_multicast_port);
    reconnect();

    // set up parameter-change callback
    define_params_change_callback_lambda_function();
    parameter_event_sub = parameters_client->on_parameter_event(params_change_callback);

    ssl_geometry_pub = this->create_publisher<pack_msgs::msg::SSLVisionGeometry>("/vision_geom", 3);
    ssl_detection_pub = this->create_publisher<pack_msgs::msg::SSLVisionDetection>("/vision_detection", 3);

    timer = this->create_wall_timer(2ms, std::bind(&VisionNode::timer_callback, this));

}

void VisionNode::timer_callback()
{
    while (vision != nullptr && vision->receive(vision_packet)) {
        if (vision_packet.has_detection()) {
            pack_msgs::msg::SSLVisionDetection::SharedPtr detection{new pack_msgs::msg::SSLVisionDetection};
            *detection = pr::convert_detection_frame(vision_packet.detection());
            detection->header.stamp = rclcpp::Node::now();
            ssl_detection_pub->publish(*detection);
        }

        if (vision_packet.has_geometry()) {
            pack_msgs::msg::SSLVisionGeometry::SharedPtr geometry{new pack_msgs::msg::SSLVisionGeometry};
            *geometry = pr::convert_geometry_data(vision_packet.geometry());
            ssl_geometry_pub->publish(*geometry);
        }
    }
}

void VisionNode::reconnect() {
    delete vision;
    vision = new RoboCupSSLClient(vision_multicast_port, vision_multicast_ip);
    if (!vision->open(false)) {
        RCLCPP_INFO(this->get_logger(), "COULDN'T stablish udp com: " + vision_multicast_ip +":%d", vision_multicast_port);
    } else {
        RCLCPP_INFO(this->get_logger(), "stablish udp com: " + vision_multicast_ip +":%d", vision_multicast_port);
    }

}

void VisionNode::define_params_change_callback_lambda_function()
{
    params_change_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
        for (auto & new_parameter : event->new_parameters) {
            //do stuff
        }
        for (auto & changed_parameter : event->changed_parameters) {
            if(changed_parameter.name == "vision_multicast_ip")
            {
                vision_multicast_ip = changed_parameter.value.string_value;
                reconnect();
            }
            else if(changed_parameter.name == "vision_multicast_port")
            {
                vision_multicast_port = changed_parameter.value.integer_value;
                reconnect();
            }
        }
        for (auto & deleted_parameter : event->deleted_parameters) {
            //do stuff
        }
    };

}
