//
// Created by Kian Behzad on 3/5/21.
//

#include "pack_ai/ai/ai_node.h"

pack_msgs::msg::WorldModel::SharedPtr extern_wm;
Drawer* extern_drawer;
SkillHandler* extern_skill_handler;
double extern_formation_acquisition_step = 4.6;
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
    extern_formation_acquisition_step = parameters_client->get_parameter("formation_acquisition_step", extern_formation_acquisition_step);
    extern_temp_value1 = parameters_client->get_parameter("temp_value1", extern_temp_value1);
    extern_temp_value2 = parameters_client->get_parameter("temp_value2", extern_temp_value2);

    // set up world_model callback
    worldmodel_subscription = this->create_subscription<pack_msgs::msg::WorldModel>("/world_model", 10, std::bind(&AINode::worldmodel_callback, this, _1));

    // set up debug draws publisher
    extern_drawer = new Drawer{this->get_name()};
    debugdraws_publisher = this->create_publisher<pack_msgs::msg::Shapes>("/debug_draws", 5);

    // set up skill publisher
    extern_skill_handler = new SkillHandler{};
    for (int i = 0; i < knowledge::MAX_ROBOT_NUM; i++)
        skill_publisher[i] = this->create_publisher<pack_msgs::msg::Skill>("/agent_"+QString::number(i).toStdString()+"/skill", 5);
}

AINode::~AINode()
{
    delete extern_drawer; extern_drawer = nullptr;

}

void AINode::worldmodel_callback(const pack_msgs::msg::WorldModel::SharedPtr msg)
{
    extern_wm = msg;

    // execute the coach
    coach.execute();

    // publish all assigned skills
    for(const auto& skill : extern_skill_handler->get_skills())
        skill_publisher[skill.id]->publish(skill);
    // publish all draws
    debugdraws_publisher->publish(extern_drawer->get_draws());
}

void AINode::define_params_change_callback_lambda_function()
{
    params_change_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
        for (auto & new_parameter : event->new_parameters) {
            //do stuff
        }
        for (auto & changed_parameter : event->changed_parameters) {
            if(changed_parameter.name == "formation_acquisition_step")
            {
                extern_formation_acquisition_step = changed_parameter.value.double_value;
                if(extern_formation_acquisition_step == 0)  extern_formation_acquisition_step = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
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
