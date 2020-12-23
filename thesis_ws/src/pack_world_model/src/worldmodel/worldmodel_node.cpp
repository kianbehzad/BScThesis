#include "pack_world_model/worldmodel/worldmodel_node.h"


// extern value definitions
CameraConfig extern_cameraConfig;
bool extern_isSimulation;
bool extern_isOurSideLeft;
bool extern_isOurColorYellow;


WorldModelNode::WorldModelNode(const rclcpp::NodeOptions & options) : Node("worldmodel_node", options)
{
    //feature initialization
    wm.reset(new WorldModel);

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

    //get initial parameters
    extern_cameraConfig.cam_num = parameters_client->get_parameter("cam_num", extern_cameraConfig.cam_num);
    extern_cameraConfig.camera_0 = parameters_client->get_parameter("camera_0", extern_cameraConfig.camera_1);
    extern_cameraConfig.camera_1 = parameters_client->get_parameter("camera_1", extern_cameraConfig.camera_1);
    extern_cameraConfig.camera_2 = parameters_client->get_parameter("camera_2", extern_cameraConfig.camera_2);
    extern_cameraConfig.camera_3 = parameters_client->get_parameter("camera_3", extern_cameraConfig.camera_3);
    extern_cameraConfig.camera_4 = parameters_client->get_parameter("camera_4", extern_cameraConfig.camera_4);
    extern_cameraConfig.camera_5 = parameters_client->get_parameter("camera_5", extern_cameraConfig.camera_5);
    extern_cameraConfig.camera_6 = parameters_client->get_parameter("camera_6", extern_cameraConfig.camera_6);
    extern_cameraConfig.camera_7 = parameters_client->get_parameter("camera_7", extern_cameraConfig.camera_7);
    extern_isSimulation = parameters_client->get_parameter("is_simulation", extern_isSimulation);
    extern_isOurSideLeft = parameters_client->get_parameter("is_our_side_left", extern_isOurSideLeft);
    extern_isOurColorYellow = parameters_client->get_parameter("is_our_color_yellow", extern_isOurColorYellow);

    //set up world-model publisher
    worldmodel_publisher = this->create_publisher<pack_msgs::msg::WorldModel>("/world_model", 5);


    // set up vision_detection callback
    vision_detection_subscription = this->create_subscription<pack_msgs::msg::SSLVisionDetection>("/vision_detection", 8, std::bind(&WorldModelNode::vision_detection_callback, this, _1));

    // set up vision_detection callback
    vision_geometry_subscription = this->create_subscription<pack_msgs::msg::SSLVisionGeometry>("/vision_geom", 8, std::bind(&WorldModelNode::vision_geometry_callback, this, _1));

    // set up agent_command callbacks
    for(int i{}; i < knowledge::MAX_ROBOT_NUM; i++)
        command_subscription[i] = this->create_subscription<pack_msgs::msg::RobotCommand>("/agent_" + std::to_string(i) + "/command", 3, std::bind(&WorldModelNode::command_callback, this, _1));

}

void WorldModelNode::vision_detection_callback(const pack_msgs::msg::SSLVisionDetection::SharedPtr msg)
{
    wm->updateDetection(msg);
    wm->execute();
    frame ++;
    packs ++;
    if (packs >= extern_cameraConfig.cam_num) {
        packs = 0;
        wm->merge(frame);
        pack_msgs::msg::WorldModel::SharedPtr temp = wm->getParsianWorldModel();
        temp->header.stamp = rclcpp::Node::now();
        temp->header.frame_id = std::to_string(msg->frame_number);
        temp->is_left = extern_isOurSideLeft;
        temp->is_yellow = extern_isOurColorYellow;
        worldmodel_publisher->publish(*temp);
    }
}

void WorldModelNode::vision_geometry_callback(const pack_msgs::msg::SSLVisionGeometry::SharedPtr msg)
{

}

void WorldModelNode::command_callback(const pack_msgs::msg::RobotCommand::SharedPtr msg)
{
    wm->vForwardCmd[msg->robot_id] = msg->vel_f;
    wm->vNormalCmd [msg->robot_id] = msg->vel_n;
    wm->vAngCmd    [msg->robot_id] = msg->vel_w;
}

void WorldModelNode::define_params_change_callback_lambda_function()
{
    params_change_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
        for (auto & new_parameter : event->new_parameters) {
            //do stuff
        }
        for (auto & changed_parameter : event->changed_parameters) {
            if(changed_parameter.name == "is_simulation")
            {
                extern_isSimulation = changed_parameter.value.bool_value;
                wm->setMode(extern_isSimulation);
            }
            else if(changed_parameter.name == "is_our_side_left")
            {
                extern_isOurSideLeft = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "is_our_color_yellow")
            {
                extern_isOurColorYellow = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "cam_num")
            {
                extern_cameraConfig.cam_num = changed_parameter.value.integer_value;
            }
            else if(changed_parameter.name == "camera_0")
            {
                extern_cameraConfig.camera_0 = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "camera_1")
            {
                extern_cameraConfig.camera_1 = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "camera_2")
            {
                extern_cameraConfig.camera_2 = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "camera_3")
            {
                extern_cameraConfig.camera_3 = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "camera_4")
            {
                extern_cameraConfig.camera_4 = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "camera_5")
            {
                extern_cameraConfig.camera_5 = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "camera_6")
            {
                extern_cameraConfig.camera_6 = changed_parameter.value.bool_value;
            }
            else if(changed_parameter.name == "camera_7")
            {
                extern_cameraConfig.camera_7 = changed_parameter.value.bool_value;
            }
        }
        for (auto & deleted_parameter : event->deleted_parameters) {
            //do stuff
        }
    };

}
