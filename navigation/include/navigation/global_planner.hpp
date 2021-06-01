
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

#include <stack>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <list>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
//#include "nav2_util/geometry_utils.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "sgd_msgs/srv/compute_path.hpp"
#include "rapidxml/rapidxml.hpp"
#include "sgd_util/sgd_util.hpp"

namespace nav_sgd
{

class Global_Planner_OSM : public nav2_util::LifecycleNode
{

//! \brief Struct to hold node id, latitude and longitude
struct NODE
{
    rapidxml::xml_node<char> *xml_node;
    rapidxml::xml_node<char> *parent_xml_node;
    double f,g,h;
};

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
    //void computePath(double lat_a, double lon_a, double lat_b, double lon_b);
    void computePath(const std::shared_ptr<sgd_msgs::srv::ComputePath::Request> request,
           std::shared_ptr<sgd_msgs::srv::ComputePath::Response> response);

protected:
    // Implement the lifecycle interface
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    //! \brief Init parameters
    void init_parameters();
    std::string map_file_;
    std::string waypoints_topic_;
    std::string mapdata_topic_;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::TimerBase::SharedPtr timer_map_data_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_map_data_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_waypoints_;

    
    // variablesused for parsing osm
    std::vector<char> buffer;
    rapidxml::xml_document<> osm;
    rapidxml::xml_node<> * root;

    // Store waypoints from start to destination
    std::vector<POSE> waypoints;

    // Store map data
    std::vector<POSE> map_data;
 
    // service
    std::chrono::milliseconds server_timeout_;
    rclcpp::Node::SharedPtr client_node_;
    rclcpp::Service<sgd_msgs::srv::ComputePath>::SharedPtr compute_path_srv;
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr waypoint_follower_action_client_;
    nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr waypoint_follower_goal_handle_;

    // publisher / subscriber
    

    //! \brief Publish map data to map_data topic
    void publish_map_data();

    //! \brief Publish waypoints after path computation
    void publish_waypoints();

    //! \brief Read xml file and parse it
    void parseXml();

    //! Check if pos is destination
    //! \param pos xml_node for current position
    //! \param dest xml_node for destination
    //! \return TRUE if node id of current position equals id of destination node
    bool isDestination(rapidxml::xml_node<char> *pos, rapidxml::xml_node<char> *dest);

    //! \brief Search nearest node to current position
    //! \param lat current latitude
    //! \param lon current longitude
    //! \return id of nearest node
    rapidxml::xml_node<char>* get_id(double const lat, double const lon);

    //! \brief Calculate distance from node start to node dest
    //! \param start start node
    //! \param dest destination node
    //! \return distance between nodes
    double distance_node_to_node(rapidxml::xml_node<char> *start, rapidxml::xml_node<char> *dest);

    //! \brief Cost function from node start to node dest.
    //! \param start start node
    //! \param dest destination node
    //! \return cost from node start to node dest
    double cost_node_to_node(rapidxml::xml_node<char> *start, rapidxml::xml_node<char> *dest);

    //! \brief
    void trace_path(std::unordered_map<rapidxml::xml_node<char> *, rapidxml::xml_node<char> *> closedList,
            rapidxml::xml_node<char> * dest);

    //! \brief
    //! \param data pointer to vector with points to publish
    //! \param r color channel red, default value 0.0
    //! \param g color channel green, default value 0.8
    //! \param b color channel blue, default value 0.0
    //! \param a color channel alpha, default value 1.0
    void publish_marker_array(std::vector<POSE> * data,
        std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> publisher,
        float size=0.5f, float r=0.0f, float g=0.8f, float b=0.0f, float a=1.0f);

    //! \brief clear all markers published by the specified publisher
    void clear_marker_array(std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> publisher);

    //! \brief Start waypoint following
    //! \param poses vector of waypoints
    void start_waypoint_following(std::vector<POSE> * waypoints);

};

}   // namespace nav_sgd



#endif
