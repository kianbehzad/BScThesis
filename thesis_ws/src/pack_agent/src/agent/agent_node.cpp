//
// Created by Kian Behzad on 1/15/21.
//

#include "pack_agent/agent/agent_node.h"

// extern value definitions
pack_msgs::msg::WorldModel::SharedPtr extern_wm;
Drawer* extern_drawer;
double extern_P_pos = 1;
double extern_I_pos = 0;
double extern_D_pos = 0;
double extern_P_angle = 1;
double extern_I_angle = 0;
double extern_D_angle = 0;
double extern_max_vel = 4;
double extern_attraction_radius = 2;
double extern_attraction_step = 1;
double extern_repulsion_static_radius = 0.5;
double extern_repulsion_static_step = 100;
double extern_repulsion_static_prediction = 0.4;
double extern_repulsion_dynamic_step = 10000;
double extern_repulsion_dynamic_prediction = 0.8;
double extern_temp_value1 = 0;
double extern_temp_value2 = 0;

AgentNode::AgentNode(const rclcpp::NodeOptions & options) : Node("agent_node", options)
{
    // set up parameter client
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "agent_param_node");
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
    extern_P_angle = parameters_client->get_parameter("P_angle", extern_P_angle);
    extern_I_angle = parameters_client->get_parameter("I_angle", extern_I_angle);
    extern_D_angle = parameters_client->get_parameter("D_angle", extern_D_angle);
    extern_P_pos = parameters_client->get_parameter("P_pos", extern_P_pos);
    extern_I_pos = parameters_client->get_parameter("I_pos", extern_I_pos);
    extern_D_pos = parameters_client->get_parameter("D_pos", extern_D_pos);
    extern_max_vel = parameters_client->get_parameter("max_vel", extern_max_vel);
    extern_attraction_radius = parameters_client->get_parameter("attraction_radius", extern_attraction_radius);
    extern_attraction_step = parameters_client->get_parameter("attraction_step", extern_attraction_step);
    extern_repulsion_static_radius = parameters_client->get_parameter("repulsion_static_radius", extern_repulsion_static_radius);
    extern_repulsion_static_step = parameters_client->get_parameter("repulsion_static_step", extern_repulsion_static_step);
    extern_repulsion_static_prediction = parameters_client->get_parameter("repulsion_static_prediction", extern_repulsion_static_prediction);
    extern_repulsion_dynamic_step = parameters_client->get_parameter("repulsion_dynamic_step", extern_repulsion_dynamic_step);
    extern_repulsion_dynamic_prediction = parameters_client->get_parameter("repulsion_dynamic_prediction", extern_repulsion_dynamic_prediction);
    extern_temp_value1 = parameters_client->get_parameter("temp_value1", extern_temp_value1);
    extern_temp_value2 = parameters_client->get_parameter("temp_value2", extern_temp_value2);

    // setup skills
    skill_gotopoint = new SkillGotoPoint{};
    skill_gotopoint_avoid = new SkillGotoPointAvoid{};
    skill_none = new SkillNone{};

    // set up world_model callback
    worldmodel_subscription = this->create_subscription<pack_msgs::msg::WorldModel>("/world_model", 10, std::bind(&AgentNode::worldmodel_callback, this, _1));

    // set up skill callback
    skill_subscription = this->create_subscription<pack_msgs::msg::Skill>("~/skill", 10, std::bind(&AgentNode::skill_callback, this, _1));

    // set up robot command publisher
    robotcommand_publisher = this->create_publisher<pack_msgs::msg::RobotCommand>("~/command", 5);

    // set up debug draws publisher
    extern_drawer = new Drawer{this->get_name()};
    debugdraws_publisher = this->create_publisher<pack_msgs::msg::Shapes>("/debug_draws", 5);

}

AgentNode::~AgentNode()
{
   delete extern_drawer; extern_drawer = nullptr;
   delete skill_gotopoint; skill_gotopoint = nullptr;
   delete skill_gotopoint_avoid; skill_gotopoint_avoid = nullptr;
   delete skill_none; skill_none = nullptr;
}

void AgentNode::worldmodel_callback(const pack_msgs::msg::WorldModel::SharedPtr msg)
{
    extern_wm = msg;

    if(skill_msg != nullptr)
    {
        switch (skill_msg->skill_type) {
            case pack_msgs::msg::Skill::SKILLGOTOPOINT:
                skill = skill_gotopoint;
            break;
            case pack_msgs::msg::Skill::SKILLGOTOPOINTAVOID:
                skill = skill_gotopoint_avoid;
            break;
            case pack_msgs::msg::Skill::NONE:
                skill = skill_none;
            break;
        }
        robotcommand_publisher->publish(skill->execute(*skill_msg));
    }

    debugdraws_publisher->publish(extern_drawer->get_draws());
}

void AgentNode::skill_callback(const pack_msgs::msg::Skill::SharedPtr msg)
{
    skill_msg = msg;
}

void AgentNode::define_params_change_callback_lambda_function()
{
    params_change_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
        for (auto & new_parameter : event->new_parameters) {
            //do stuff
        }
        for (auto & changed_parameter : event->changed_parameters) {
            if(changed_parameter.name == "P_angle")
            {
                extern_P_angle = changed_parameter.value.double_value;
                if(extern_P_angle == 0)  extern_P_angle = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "I_angle")
            {
                extern_I_angle = changed_parameter.value.double_value;
                if(extern_I_angle == 0)  extern_I_angle = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "D_angle")
            {
                extern_D_angle = changed_parameter.value.double_value;
                if(extern_D_angle == 0)  extern_D_angle = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
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
            else if(changed_parameter.name == "attraction_radius")
            {
                extern_attraction_radius = changed_parameter.value.double_value;
                if(extern_attraction_radius == 0)  extern_attraction_radius = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "attraction_step")
            {
                extern_attraction_step = changed_parameter.value.double_value;
                if(extern_attraction_step == 0)  extern_attraction_step = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "repulsion_static_radius")
            {
                extern_repulsion_static_radius = changed_parameter.value.double_value;
                if(extern_repulsion_static_radius == 0)  extern_repulsion_static_radius = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "repulsion_static_step")
            {
                extern_repulsion_static_step = changed_parameter.value.double_value;
                if(extern_repulsion_static_step == 0)  extern_repulsion_static_step = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "repulsion_static_prediction")
            {
                extern_repulsion_static_prediction = changed_parameter.value.double_value;
                if(extern_repulsion_static_prediction == 0)  extern_repulsion_static_prediction = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "repulsion_dynamic_step")
            {
                extern_repulsion_dynamic_step = changed_parameter.value.double_value;
                if(extern_repulsion_dynamic_step == 0)  extern_repulsion_dynamic_step = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "repulsion_dynamic_prediction")
            {
                extern_repulsion_dynamic_prediction = changed_parameter.value.double_value;
                if(extern_repulsion_dynamic_prediction == 0)  extern_repulsion_dynamic_prediction = changed_parameter.value.integer_value;//double_value gives 0 if the input has no decimals
            }
            else if(changed_parameter.name == "temp_value1")
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
