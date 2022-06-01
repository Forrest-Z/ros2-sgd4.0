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
    declare_parameter("goal_pose_topic", rclcpp::ParameterValue("goalpose"));
    declare_parameter("route_info_topic", rclcpp::ParameterValue("route_info"));
    declare_parameter("light_topic", rclcpp::ParameterValue("lights"));

    get_parameter("goal_pose_topic", goal_pose_topic_);
    get_parameter("route_info_topic", route_info_topic_);
    get_parameter("light_topic", light_topic_);

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
    init_maneuvers();
}

Master_Control_Unit::~Master_Control_Unit() {}

void
Master_Control_Unit::init_pub_sub()
{
    //pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("sgd_move_base", default_qos);
    sub_goal_pose_ = this->create_subscription<geometry_msgs::msg::Point>(goal_pose_topic_, default_qos,
                        std::bind(&Master_Control_Unit::on_goalpose_received, this, std::placeholders::_1));
    
    sub_route_info_ = this->create_subscription<sgd_msgs::msg::RouteInfo>(route_info_topic_, default_qos,
                        std::bind(&Master_Control_Unit::on_route_info_received, this, std::placeholders::_1));

    //pub_light_ = this->create_publisher<sgd_msgs::msg::Light>(light_topic_, default_qos);
}

void
Master_Control_Unit::init_maneuvers()
{
    maneuvers_.insert({-2.618, "Nach rechts umkehren in "});
    maneuvers_.insert({-2.094, "Stark rechts abbiegen in "});
    maneuvers_.insert({-1.047, "Rechts abbiegen in "});
    maneuvers_.insert({-0.349, "Leicht rechts in "});
    maneuvers_.insert({0.349, "Geradeaus in "});
    maneuvers_.insert({1.047, "Leicht links in "});
    maneuvers_.insert({2.094, "Links abbiegen in "});
    maneuvers_.insert({2.618, "Stark links abbiegen in "});
    maneuvers_.insert({3.142, "Nach links umkehren in "});
    maneuvers_.insert({4.0, "Ziel erreicht in "});
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

void
Master_Control_Unit::on_route_info_received(const sgd_msgs::msg::RouteInfo::SharedPtr msg_)
{
    auto it = maneuvers_.begin();
    while (msg_->angle > it->first && it != maneuvers_.end())
    {
        it++;
    }
    RCLCPP_INFO(get_logger(), "Next maneuver: %s (angle: %.2f) in %i m", it->second.c_str(), msg_->angle, msg_->distance);

    // TODO control lights

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
