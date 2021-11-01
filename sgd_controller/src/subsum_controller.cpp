
#include "sgd_controller/subsum_controller.hpp"

namespace nav_sgd
{

using std::placeholders::_1;
using namespace std::chrono_literals; 

Subsum_Controller::Subsum_Controller() 
        : nav2_util::LifecycleNode("subsum_controller", "", true),
          in_topics_{"cmd_vel_lidar", "cmd_vel_nav2"}
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    // Add parameters

    declare_parameter("move_topics", in_topics_);
    declare_parameter("sgd_move_topic", rclcpp::ParameterValue("sgd_move_base"));

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
    //init_parameters();
    get_parameter("move_topics", in_topics_);
    get_parameter("sgd_move_topic", out_topic_);

    if (in_topics_.size() < 1 || out_topic_.length() < 1)
    {
        RCLCPP_ERROR(get_logger(), "Could not initialize input or output topics.");
        return nav2_util::CallbackReturn::FAILURE;
    }

    init_pub_sub();

    active_layer = in_topics_.size() - 1;
    for (uint8_t i = 0; i < in_topics_.size(); i++)
    {
        last_time_received_.push_back(0.0);
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
    pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("sgd_move_base", default_qos);

    for (uint8_t i = 0; i < in_topics_.size(); i++)
    {
        std::function<void(std::shared_ptr<geometry_msgs::msg::Twist>)> fnc = std::bind(
            &Subsum_Controller::on_cm_vel_received, this, std::placeholders::_1, (int)i);
        //sub_input_ = node->create_subscription<sensor_msgs::msg::Imu>("input", fnc);
        RCLCPP_INFO(get_logger(), "Create subscription for topic %s on layer %i", in_topics_.at(i).c_str(), i);
        subscriber.push_back(this->create_subscription<geometry_msgs::msg::Twist>(in_topics_.at(i), default_qos,
                            fnc));
    }
}

void
Subsum_Controller::on_cm_vel_received(const geometry_msgs::msg::Twist::SharedPtr msg, int layer)
{
    double t = now().nanoseconds() / 1000000.0; // current time in milliseconds
    last_time_received_[layer] = t;

    // get currently active layer
    for (int8_t i = 0; i <= layer; i++)
    {
        //RCLCPP_INFO(get_logger(), "Layer: %i, Last received: %f, Time: %f", layer, last_time_received_[i], t);
        // check time since last msg received
        if (t - last_time_received_[i] < 1000)   // time in milliseconds
        {
            if (i != active_layer)  RCLCPP_INFO(get_logger(), "Change active layer to layer %i", i);
            active_layer = i;
            break;
        }
    }

    if (layer <= active_layer)
    {
        if (layer != active_layer)  RCLCPP_INFO(get_logger(), "Change active layer to layer %i", layer);
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
