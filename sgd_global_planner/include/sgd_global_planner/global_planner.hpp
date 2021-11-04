
#ifndef GLOBAL_PLANNER_OSM_HPP_
#define GLOBAL_PLANNER_OSM_HPP_

#include <string>
#include <memory>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <iomanip>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <stack>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <list>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "sgd_msgs/srv/compute_path.hpp"
#include "sgd_util/sgd_util.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav_sgd
{

class Global_Planner_OSM : public nav2_util::LifecycleNode
{

//! \brief Struct to hold position information latitude, longitude and angle around z in radians.
struct POSE
{
    double lat, lon, angle;
};

public:
    Global_Planner_OSM();
    ~Global_Planner_OSM();

    void CreateService();

    //! \brief Compute path from A to B using A* algorithm
    //! \param lat_a Latitude of point A
    //! \param lon_a Longitude of point A
    //! \param lat_b Latitude of point B
    //! \param lon_b Longitude of point B
    void computePath(const std::shared_ptr<geometry_msgs::msg::PointStamped> msg);

protected:
    // Implement the lifecycle interface
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    //! \brief Init parameters
    void init_parameters();
    std::string waypoints_topic_;
    std::string clicked_point_topic_;
    int port_;
    std::string ip_address_;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr publisher_path_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_clicked_point_;

    //! \brief Init transforms
    void init_transforms();
    nav2_util::CallbackReturn wait_for_transform();
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped tf_map_odom_;

    //! \brief Initialize connection to path planning server
    void init_ip_connection();
    int sock;

    // Store waypoints from start to destination
    std::vector<POSE> get_waypoints_from_msg(std::string waypoints);
    //std::vector<POSE> waypoints;

    // service
    std::chrono::milliseconds server_timeout_;
    rclcpp::Node::SharedPtr client_node_;
    rclcpp::Service<sgd_msgs::srv::ComputePath>::SharedPtr compute_path_srv;
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr waypoint_follower_action_client_;
    nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr waypoint_follower_goal_handle_;

    //! \brief Publish map data to map_data topic
    void publish_map_data();

    //! \brief Publish waypoints after path computation
    void publish_waypoints();

    //! \brief
    //! \param data pointer to vector with points to publish
    void publish_marker_array(std::vector<POSE> * data,
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> publisher);
    
    //! \brief Start waypoint following
    //! \param poses vector of waypoints
    void start_waypoint_following(std::vector<POSE> * waypoints);

    //! \brief Convert double to string with high precision
	std::string to_string( double d ) {
        std::ostringstream stm;
        stm << std::setprecision(10) << d;
        return stm.str() ;
    }
};

}   // namespace nav_sgd



#endif
