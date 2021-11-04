
#include "sgd_global_planner/global_planner.hpp"

namespace nav_sgd
{

using std::placeholders::_1;
using namespace std::chrono_literals;

Global_Planner_OSM::Global_Planner_OSM():
        nav2_util::LifecycleNode("global_planner_osm", "", true) 
{   
    RCLCPP_DEBUG(get_logger(), "Creating");

    // Add parameters
    declare_parameter("waypoints_topic", rclcpp::ParameterValue("global_plan"));
    declare_parameter("clicked_point_topic", rclcpp::ParameterValue("clicked_point"));
    declare_parameter("port", rclcpp::ParameterValue(8080));
    declare_parameter("ip_address", rclcpp::ParameterValue("127.0.0.1"));
}

Global_Planner_OSM::~Global_Planner_OSM()
{
    // Destroy
}

nav2_util::CallbackReturn
Global_Planner_OSM::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    init_pub_sub();
    init_transforms();
    init_ip_connection();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Global_Planner_OSM::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Activating");
    publisher_path_->on_activate();

    return wait_for_transform();
}

nav2_util::CallbackReturn
Global_Planner_OSM::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Global_Planner_OSM::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Global_Planner_OSM::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void
Global_Planner_OSM::init_parameters()
{
    get_parameter("waypoints_topic", waypoints_topic_);
    get_parameter("clicked_point_topic", clicked_point_topic_);
    get_parameter("port", port_);
    get_parameter("ip_address", ip_address_);
}

void
Global_Planner_OSM::init_pub_sub()
{
    publisher_path_ = this->create_publisher<nav_msgs::msg::Path>(waypoints_topic_, default_qos);

    sub_clicked_point_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        clicked_point_topic_, default_qos, std::bind(&Global_Planner_OSM::computePath, this, _1));

    RCLCPP_DEBUG(get_logger(), "Initialised publisher on topic %s and subscriber on %s.",
            waypoints_topic_.c_str(), clicked_point_topic_.c_str());

    auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args --remap __node:=navigation_dialog_action_client"});
    client_node_ = std::make_shared<rclcpp::Node>("_follow", options);
    waypoint_follower_action_client_ = 
        rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
        client_node_,
        "FollowWaypoints");
    waypoint_follower_goal_ = nav2_msgs::action::FollowWaypoints::Goal();

    server_timeout_ = std::chrono::milliseconds(10);
}

void
Global_Planner_OSM::init_transforms()
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rclcpp_node_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        rclcpp_node_->get_node_base_interface(),
        rclcpp_node_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void
Global_Planner_OSM::init_ip_connection()
{
    struct sockaddr_in serv_addr;

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        RCLCPP_ERROR(get_logger(), "Socket creation error!");
        return;
    }
   
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port_);
       
    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, ip_address_.c_str(), &serv_addr.sin_addr)<=0) 
    {
        RCLCPP_ERROR(get_logger(), "Invalid address/ Address not supported");
        return;
    }
   
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        RCLCPP_ERROR(get_logger(), "Connection Failed!");
        return;
    }
}

nav2_util::CallbackReturn
Global_Planner_OSM::wait_for_transform()
{
    // Wait for transform to be available
    RCLCPP_DEBUG(get_logger(), "Wait for transform");

    std::string err;
    int retries = 0;
    while (rclcpp::ok() && !tf_buffer_->canTransform("base_link", "map", tf2::TimePointZero, tf2::durationFromSec(0.1), &err)
        && retries < 10)
    {
        RCLCPP_INFO(this->get_logger(), "Timeout waiting for transform. Tf error: %s", err);
        err.clear();
        rclcpp::sleep_for(500000000ns);
        retries++;
    }
    return (retries > 9 ? nav2_util::CallbackReturn::FAILURE : nav2_util::CallbackReturn::SUCCESS);
}

void
Global_Planner_OSM::computePath(const std::shared_ptr<geometry_msgs::msg::PointStamped> msg)
{
    // get current position
    double x_base_map, y_base_map;
    try
    {
        geometry_msgs::msg::TransformStamped tf_base_map = tf_buffer_->lookupTransform("map", "base_link",
                    rclcpp::Time(0), rclcpp::Duration(5,0));
        x_base_map = tf_base_map.transform.translation.x;
        y_base_map = tf_base_map.transform.translation.y;
        RCLCPP_INFO(this->get_logger(), "Transform map -> base_link x: %f, %f", x_base_map, y_base_map);
    } catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        return;
    }

    std::vector<std::string> in;
    in.push_back(""); // TODO get username
    // local coordinates to lat/lon
    auto latlon = sgd_util::local_to_WGS84(x_base_map, y_base_map);
    RCLCPP_INFO(get_logger(), "Current pos (lat/lon): %.7f, %.7f", latlon.first, latlon.second);
    in.push_back(to_string(latlon.first));
    in.push_back(to_string(latlon.second));

    // get destination from message
    auto dest = sgd_util::local_to_WGS84(msg->point.x, msg->point.y);
    in.push_back(to_string(dest.first));
    in.push_back(to_string(dest.second));

    // build message
    /*
    {
      "mode": 0,
      "username": "default",
      "start": [53.1234567, 10.1234567],
      "dest": [53.9876543, 10.9876543],
      "address": "Berliner Tor 21"
    }
    */
    std::string s = "{\n  \"mode\": 0,\n  \"username\":\"default\",\n  \"start\": [";
    s.append(to_string(latlon.first) + "," + to_string(latlon.second) + "],\n");
    s.append("  \"dest\": [" + to_string(dest.first) + "," + to_string(dest.second) + "],\n");
    s.append("  \"address\": \"\"\n}");
    
    //RCLCPP_INFO(get_logger(), "Send message %s", s.c_str());
    send(sock , s.c_str() , s.length() , 0 );
    s.clear();

    // wait for response
    std::string wps;
    std::vector<POSE> waypoints;
    bool IN_MSG = false;
    while (1)
    {
        char c;
        read(sock , &c, 1);

        if (c == '{' && !IN_MSG)
        {
            wps.clear();
            IN_MSG = true;
        }
        else if (IN_MSG && c == '}')
        {
            // end of message -> parse msg, compute waypoints and send to controller
            waypoints = get_waypoints_from_msg(wps);
            if (waypoints.size() < 1)
            {
                RCLCPP_INFO(get_logger(), "No waypoints received!");
            }
            else
            {
                publish_marker_array(&waypoints, publisher_path_);
                start_waypoint_following(&waypoints);
            }
            
            return;
        }
        else if (IN_MSG)
        {
            wps.push_back(c);
        }
    }
}

void
Global_Planner_OSM::publish_marker_array(
    std::vector<POSE> * data,
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> publisher)
{
    std::vector<POSE>::iterator it;
    nav_msgs::msg::Path path_;

    path_.header.frame_id = "map";
    path_.header.stamp = this->get_clock()->now();

    geometry_msgs::msg::PoseStamped last_p_;
    tf2::Quaternion angle_to_last_p_;
    for (auto p : *data)
    {
        auto xy = sgd_util::WGS84_to_local(p.lat, p.lon);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_.header;

        pose.pose.position.x = xy.first;
        pose.pose.position.y = xy.second;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        double angle = atan2(pose.pose.position.y - last_p_.pose.position.y,
                    pose.pose.position.x - last_p_.pose.position.x);
        angle_to_last_p_.setRPY(0, 0, angle);
        last_p_ = pose;
        path_.poses.push_back(pose);
    }

    RCLCPP_INFO(get_logger(), "Angle from last point: %f", angle_to_last_p_.getAngle());
    path_.poses.back().pose.orientation = tf2::toMsg(angle_to_last_p_);
    publisher->publish(path_);
}

void
Global_Planner_OSM::start_waypoint_following(std::vector<POSE> * waypoints)
{
    auto is_action_server_ready = 
        waypoint_follower_action_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!is_action_server_ready)
    {
        RCLCPP_ERROR(this->get_logger(), "FollowWaypoints action server is not available.");
        return;
    }

    std::vector<POSE>::iterator it;
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    geometry_msgs::msg::PoseStamped ps;

    for (it = waypoints->begin(); it != waypoints->end(); ++it)
    {
        ps = geometry_msgs::msg::PoseStamped();

        auto p = *it;
        auto xy = sgd_util::WGS84_to_local(p.lat, p.lon);

        ps.header.stamp = rclcpp::Clock().now();
        ps.header.frame_id = "map";
        ps.pose.position.x = xy.first;
        ps.pose.position.y = xy.second;
        ps.pose.position.z = 0.0;
        ps.pose.orientation = sgd_util::rotation_around_z(p.angle);

        poses.push_back(ps);
    }

    waypoint_follower_goal_.poses = poses;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
    send_goal_options.result_callback = [](auto) {};

    auto future_goal_handle = waypoint_follower_action_client_->async_send_goal(waypoint_follower_goal_, send_goal_options);
    if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Send goal call failed.");
        return;
    }

    waypoint_follower_goal_handle_ = future_goal_handle.get();
    if (!waypoint_follower_goal_handle_)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
        return;
    }
}

std::vector<Global_Planner_OSM::POSE>
Global_Planner_OSM::get_waypoints_from_msg(std::string waypoints_)
{
    std::vector<POSE> waypoints;

    std::string wps_str = waypoints_.substr(waypoints_.find_first_of("[")+1,waypoints_.find_last_of("]")-1);
    RCLCPP_INFO(get_logger(), "Parse message %s", wps_str.c_str());

    std::string token;
    std::istringstream tokenStream(wps_str);
    while (std::getline(tokenStream, token, ']'))
    {
        if (token.size() < 5)   continue;
        token = token.substr(token.find_first_of("[")+1);

        int del_pos = token.find(',');
        POSE p;
        p.lat = stod(token.substr(0, del_pos));
        p.lon = stod(token.substr(del_pos + 1));
        
        waypoints.push_back(p);
    }
    
    return waypoints;
}

}   // namespace nav_sgd

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nav_sgd::Global_Planner_OSM>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}

