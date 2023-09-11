
#include "sgd_lifecycle_manager/exit_publisher.hpp"

namespace sgd_lifecycle
{

ExitPublisher::ExitPublisher()
    : Node("exit_publisher")
{
    // declare and get parameter
    auto ndname_ = declare_parameter<std::string>("ndname", "");

    // create publisher
    auto pub = this->create_publisher<lifecycle_msgs::msg::TransitionEvent>("node_exit", rclcpp::QoS(rclcpp::SensorDataQoS()));
    
    // create message
    lifecycle_msgs::msg::Transition msg;
    msg.id = lifecycle_msgs::msg::Transition::TRANSITION_DESTROY;
    msg.label = ndname_;

    lifecycle_msgs::msg::TransitionEvent ev_msg;
    ev_msg.transition = msg;
    ev_msg.goal_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;
    pub->publish(ev_msg);
}

ExitPublisher::~ExitPublisher()
{
}

} // namespace sgd_lifecycle

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_lifecycle::ExitPublisher>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
