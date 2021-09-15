
#include "sgd_controller/subsum_controller.hpp"

namespace nav_sgd
{

using std::placeholders::_1;
using namespace std::chrono_literals; 

Subsum_Controller::Subsum_Controller() 
        : nav2_util::LifecycleNode("subsum_controller", "", true)
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    // Add parameters
    add_parameter("topic_layer0", rclcpp::ParameterValue(""));
    add_parameter("topic_layer1", rclcpp::ParameterValue(""));
    add_parameter("topic_layer2", rclcpp::ParameterValue(""));
    add_parameter("topic_layer3", rclcpp::ParameterValue(""));
    add_parameter("topic_layer4", rclcpp::ParameterValue(""));
    add_parameter("topic_layer5", rclcpp::ParameterValue(""));
    add_parameter("topic_layer6", rclcpp::ParameterValue(""));
    add_parameter("topic_layer7", rclcpp::ParameterValue(""));
    add_parameter("topic_layer8", rclcpp::ParameterValue(""));
    add_parameter("topic_layer9", rclcpp::ParameterValue(""));

}

Subsum_Controller::~Subsum_Controller()
{
    // Destroy
}

nav2_util::CallbackReturn
Subsum_Controller::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configure");

    // init
    init_parameters();
    init_pub_sub();

    active_layer = NUM_LAYERS - 1;
    for (uint8_t i = 0; i < NUM_LAYERS; i++)
    {
        last_time_received[i] = 0.0;
    }

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Subsum_Controller::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activate");

    pub_cmd_vel->on_activate();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Subsum_Controller::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivate");
    pub_cmd_vel->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Subsum_Controller::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Subsum_Controller::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void
Subsum_Controller::init_pub_sub()
{
    pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_veltrn", default_qos);

    for (uint8_t i = 0; i < layer_topics.size(); i++)
    {   
        std::function<void(std::shared_ptr<geometry_msgs::msg::Twist>)> fnc = std::bind(
            &Subsum_Controller::on_cm_vel_received, this, std::placeholders::_1, (int)i);
        //sub_input_ = node->create_subscription<sensor_msgs::msg::Imu>("input", fnc);
        subscriber.push_back(this->create_subscription<geometry_msgs::msg::Twist>(layer_topics.at(i), default_qos,
                            fnc));
    }
}

void
Subsum_Controller::init_parameters()
{
    for (uint8_t i = 0; i < NUM_LAYERS; i++)
    {
        std::string param;
        get_parameter("topic_layer" + std::to_string(i), param);

        if (param.length() > 1)
        {
            layer_topics.push_back(param);
        }
    }
}

void
Subsum_Controller::on_cm_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg, int layer)
{
    double t = now().nanoseconds() / 1000.0;
    last_time_received[layer] = t;   // save time in milliseconds

    // get currently active layer
    for (int8_t i = 0; i < layer; i++)
    {
        // check time since last msg received
        if (t - last_time_received[i] < 0.1)
        {
            active_layer = i;
            break;
        }
    }

    if (layer <= active_layer)
    {
        active_layer = layer;
        pub_cmd_vel->publish(*msg);
    }
}

}   // namespace sgd_lc

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<nav2_util::LifecycleNode> node = std::make_shared<nav_sgd::Subsum_Controller>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Subsumption controller startup completed.");

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
