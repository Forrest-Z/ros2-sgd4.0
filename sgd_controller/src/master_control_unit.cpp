#include "sgd_controller/master_control_unit.hpp"

namespace sgd_ctrl
{

using std::placeholders::_1;
using namespace std::chrono_literals;

Master_Control_Unit::Master_Control_Unit()
        : rclcpp::Node("master_control_unit")
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    // Add parameters
    declare_parameter("move_topics", in_topics_);
    declare_parameter("sgd_move_topic", rclcpp::ParameterValue("sgd_move_base"));

    // Was soll diese Node alles können?
    // - handling von user requests
    //   - Berechnung der Route nach zieleingabe
    //   - Start / Stop der Routenführung
    // - Überwachung des sgd
    // - light controller

    // create waypoint follower action
    auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args --remap __node:=navigation_dialog_action_client"});
    client_node_ = std::make_shared<rclcpp::Node>("_navpose", options);
    nav_to_pose_action_client_ = 
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this,
        "navigate_to_pose");
    nav_to_pose_goal_ = nav2_msgs::action::NavigateToPose::Goal();

    init_pub_sub();

}

Master_Control_Unit::~Master_Control_Unit() {}

void
Master_Control_Unit::init_pub_sub()
{
    //pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("sgd_move_base", default_qos);
    subscriber = this->create_subscription<geometry_msgs::msg::Point>("goalpose", default_qos,
                        std::bind(&Master_Control_Unit::on_goalpose_received, this, std::placeholders::_1));
    
}

void
Master_Control_Unit::on_goalpose_received(const geometry_msgs::msg::Point::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Goalpose received.");
    auto is_action_server_ready = 
        nav_to_pose_action_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!is_action_server_ready)
    {
        RCLCPP_ERROR(this->get_logger(), "NavToPose action server is not available.");
        return;
    }

    // fill action data
    nav_to_pose_goal_.pose.pose.position = *msg;
    nav_to_pose_goal_.pose.header.frame_id = "map";

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [](auto) {};

    auto future_goal_handle = nav_to_pose_action_client_->async_send_goal(nav_to_pose_goal_, send_goal_options);
    if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Send goal call failed.");
        return;
    }

    nav_to_pose_goal_handle_ = future_goal_handle.get();
    if (!nav_to_pose_goal_handle_)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
        return;
    }
}

}   // namespace sgd_ctrl

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<sgd_ctrl::Master_Control_Unit>();

    rclcpp::spin(node->get_node_base_interface());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Master control unit controller startup completed.");
    rclcpp::shutdown();
    return 0;
}
