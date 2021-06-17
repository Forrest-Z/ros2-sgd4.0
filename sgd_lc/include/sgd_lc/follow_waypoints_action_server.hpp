#ifndef NAV_SGD_FOLLOW_WAYPOINTS_ACTION_SERVER_HPP_
#define NAV_SGD_FOLLOW_WAYPOINTS_ACTION_SERVER_HPP_

#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp" 
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "sgd_msgs/msg/speed_accel.hpp";

namespace nav_sgd
{

class Follow_Waypoints_Action_Server : public nav2_util::LifecycleNode
{
public:
    using WaypointFollower = nav2_msgs::action::FollowWaypoints;
    using GoalHandleWaypointFollower = rclcpp_action::ServerGoalHandle<WaypointFollower>;

    explicit Follow_Waypoints_Action_Server(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : nav2_util::LifecycleNode("follow_waypoints_action_server", "", true, options)
    {
        using namespace std::placeholders;

        add_parameter("rate", rclcpp::ParameterValue(10));
        add_parameter("out_topic", rclcpp::ParameterValue("cmd_vel"));
        add_parameter("speed_kp", rclcpp::ParameterValue(0.005));
        add_parameter("speed_ki", rclcpp::ParameterValue(0.2));
        add_parameter("turn_kp", rclcpp::ParameterValue(0.3));
        add_parameter("turn_ki", rclcpp::ParameterValue(0.1));
        add_parameter("max_speed", rclcpp::ParameterValue(0.3));    // m/s
        add_parameter("max_rot_speed", rclcpp::ParameterValue(0.2)); // rad/s
        add_parameter("max_accel", rclcpp::ParameterValue(0.2));    // m/s^2
        add_parameter("xy_goal_tolerance", rclcpp::ParameterValue(0.5));

        this->action_server_ = rclcpp_action::create_server<WaypointFollower>(
            this,
            "follow_waypoints",
            std::bind(&Follow_Waypoints_Action_Server::handle_goal, this, _1, _2),
            std::bind(&Follow_Waypoints_Action_Server::handle_cancel, this, _1),
            std::bind(&Follow_Waypoints_Action_Server::handle_accepted, this, _1));
    }

    //Follow_Waypoints_Action_Server();
    ~Follow_Waypoints_Action_Server();

protected:
    // Implement the lifecycle interface
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    //! \brief Init parameters
    void init_parameters();
    int rate_;
    std::string out_topic_;
    double speed_kp_, speed_ki_;
    double turn_kp_, turn_ki_;
    double max_speed_, max_rot_speed_, max_accel_;
    double xy_goal_tolerance_;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr pub_motor_;
    rclcpp::Subscription<sgd_msgs::msg::SpeedAccel>::SharedPtr sub_position;
    sgd_msgs::msg::SpeedAccel last_spdacc_;
    //rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_laser_;
    //rclcpp::Subscription<sgd_msgs::msg::Touch>::SharedPtr sub_touch_;

    //! \brief Init transforms
    void init_transforms();
    nav2_util::CallbackReturn wait_for_transform();
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped tf_map_odom_;

    rclcpp_action::Server<WaypointFollower>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const WaypointFollower::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleWaypointFollower> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleWaypointFollower> goal_handle);

    void execute(const std::shared_ptr<GoalHandleWaypointFollower> goal_handle);

    void publish_motordata(double vel, double turn);
    void on_position_received(const sgd_msgs::msg::SpeedAccel::SharedPtr msg);

    double distance(double curr_x, double curr_y, double ref_x, double ref_y)
    {
        return sqrt(pow(curr_x-ref_x, 2) + pow(curr_y-ref_y, 2));
    }
};

} // namespace nav_sgd

#endif
