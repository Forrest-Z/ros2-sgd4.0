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

    // Workaround for teb planner
    // get global plan from / global_plan_sgd
    // get goal pose from /goalpose and start route computation (/goalpose -> /goalpose_sgd)
    // publish next goal
    // subscriber to /plan
}

Master_Control_Unit::~Master_Control_Unit() {}

void
Master_Control_Unit::init_pub_sub()
{
    sub_goal_pose_ = this->create_subscription<geometry_msgs::msg::Point>(goal_pose_topic_, default_qos,
                std::bind(&Master_Control_Unit::on_goalpose_received, this, std::placeholders::_1));

    sub_route_info_ = this->create_subscription<sgd_msgs::msg::RouteInfo>(route_info_topic_, default_qos,
                std::bind(&Master_Control_Unit::on_route_info_received, this, std::placeholders::_1));

    sub_global_plan = this->create_subscription<nav_msgs::msg::Path>("global_plan_sgd", default_qos,
                std::bind(&Master_Control_Unit::on_global_plan_received, this, std::placeholders::_1));

    // pub_light_ = this->create_publisher<sgd_msgs::msg::Light>(light_topic_, default_qos);
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
    
    goalpose_ = *msg;   // set goalpose of global plan
    next_wp_ = 0;

    // check battery state

    // send first point from global plan to action server
    if (global_plan.poses.size() > 0)
    {
        send_goal_action(global_plan.poses.at(next_wp_).pose.position);
    }
    else
    {
        send_goal_action(goalpose_);
    }
}

void
Master_Control_Unit::send_goal_action(const geometry_msgs::msg::Point pnt)
{
    auto is_action_server_ready =
        nav_to_pose_action_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!is_action_server_ready)
    {
        RCLCPP_ERROR(this->get_logger(), "NavToPose action server is not available.");
        return;
    }

    // fill action data
    nav_to_pose_goal_.pose.pose.position = pnt;
    nav_to_pose_goal_.pose.header.frame_id = "map";

    // define callbacks and send goal to action server
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&Master_Control_Unit::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&Master_Control_Unit::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&Master_Control_Unit::result_callback, this, std::placeholders::_1);
    nav_to_pose_action_client_->async_send_goal(nav_to_pose_goal_, send_goal_options);
}

void
Master_Control_Unit::goal_response_callback(std::shared_future<Nav2Pose_GoalHandle::SharedPtr> future)
{
    auto goal_handle = future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void
Master_Control_Unit::feedback_callback(Nav2Pose_GoalHandle::SharedPtr,
                                        const std::shared_ptr<const nav2_msgs::action::NavigateToPose_Feedback> feedback)
{
    // Was kann ich hier tun?
//     RCLCPP_INFO(get_logger(), "Distance remaining: %.2f, navigation time: %d",
//             feedback->distance_remaining, feedback->navigation_time.sec);
}

void
Master_Control_Unit::result_callback(const Nav2Pose_GoalHandle::WrappedResult &result)
{
    if (next_wp_+1 < global_plan.poses.size())
    {
        RCLCPP_INFO(this->get_logger(), "Send new goal to action server");
        send_goal_action(global_plan.poses.at(++next_wp_).pose.position);
    }

    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal was successful");
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
}

void
Master_Control_Unit::on_global_plan_received(const nav_msgs::msg::Path::SharedPtr path)
{
    global_plan = *path;

    RCLCPP_INFO(this->get_logger(), "MCU received global plan with %d waypoints", global_plan.poses.size());
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

} // namespace sgd_ctrl

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<sgd_ctrl::Master_Control_Unit>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
