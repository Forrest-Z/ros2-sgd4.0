
#include "navigation/global_planner.hpp"

namespace nav_sgd
{

using std::placeholders::_1;
using namespace std::chrono_literals;

Global_Planner_OSM::Global_Planner_OSM():
        Node("global_planner_osm")
{
    RCLCPP_INFO(this->get_logger(), "Global planner initialization.");
    // TODO: read path to map file from params
    map_file_ = "/home/ipp/dev_ws/src/ros2-sgd4.0/navigation/maps/20_NavigationsFaehigeDaten.osm";
    parseXml();
    // Create publisher for map_data
    
    compute_path_srv = this->create_service<sgd_msgs::srv::ComputePath>("/compute_path",
        std::bind(&Global_Planner_OSM::computePath, this, std::placeholders::_1, std::placeholders::_2));

    publisher_map_data_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mapdata", default_qos);
    timer_map_data_ = this->create_wall_timer(5000ms, std::bind(&Global_Planner_OSM::publish_map_data, this));

    //waypoints.push_back(std::make_pair(53.555833,10.021944));
    publisher_waypoints_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints", default_qos);
    //publish_waypoints();
    //timer_waypoints_ = this->create_wall_timer(1000ms, std::bind(&Global_Planner_OSM::publish_waypoints, this));

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

Global_Planner_OSM::~Global_Planner_OSM()
{
    // Destroy
}

void
Global_Planner_OSM::computePath(const std::shared_ptr<sgd_msgs::srv::ComputePath::Request> request,
           std::shared_ptr<sgd_msgs::srv::ComputePath::Response> response)
{   
    RCLCPP_INFO(this->get_logger(), "Compute path from %.7f, %.7f to %.7f, %.7f.",
            request->lata, request->lona, request->latb, request->lonb);

    clear_marker_array(publisher_waypoints_);

    rapidxml::xml_node<char> *start_node = get_id(request->lata, request->lona);
    rapidxml::xml_node<char> *dest_node = get_id(request->latb, request->lonb);

    auto compare = [](NODE a, NODE b) {return a.f > b.f;};
    std::priority_queue<NODE, std::vector<NODE>, decltype(compare)> openList(compare);
    std::unordered_map<rapidxml::xml_node<char> *, rapidxml::xml_node<char> *> closedList;

    // Initialise parameters of starting node
    NODE d;
    d.xml_node = start_node;
    d.parent_xml_node = NULL;
    d.f = 0.0;
    d.g = 0.0;  // bisherige Kosten
    d.h = 0.0;  // estimated cost to destination
    openList.push(d);

    // initially the destination is not reached
    bool foundDest = false;

    while (!openList.empty())
    {
        NODE olistData = openList.top();

        rapidxml::xml_node<char> *pos = olistData.xml_node;

        // Remove this node from the open list
        openList.pop();

        if (isDestination(pos, dest_node))
        {
            RCLCPP_INFO(this->get_logger(), "The destination cell is found. Node ID: %s, lat: %s, lon: %s\n",
                dest_node->first_attribute("id")->value(),
                dest_node->first_attribute("lat")->value(),
                dest_node->first_attribute("lon")->value());

            closedList.insert(std::make_pair(olistData.xml_node, olistData.parent_xml_node));
            foundDest = true;
            trace_path(closedList, dest_node);

            break;
        }

        for (rapidxml::xml_node<> *nd = pos->first_node("nd"); nd; nd = nd->next_sibling("nd"))
        {
            long pos_id = strtol(nd->first_attribute("ref")->value(), NULL, 10);

            for (rapidxml::xml_node<> *np = root->first_node("node"); np; np = np->next_sibling())
            {
                if ( strtol(np->first_attribute("id")->value(), NULL, 10) == pos_id )
                {
                    // If successor is already on the closed list ignore it.
                    if (closedList.find(np) != closedList.end())
                    {
                        continue;
                    }

                    NODE n;
                    n.xml_node = np;
                    n.parent_xml_node = olistData.xml_node;
                    n.g = olistData.g + cost_node_to_node(olistData.xml_node, np);
                    n.h = distance_node_to_node(np, dest_node);
                    n.f = n.g + n.h;

                    //RCLCPP_DEBUG(this->get_logger(), "Node %d to open list, cost from start :%.8f, cost to dest: %.8f ", pos_id, n.g, n.h);
                    openList.push(n);

                    break;
                }
            }
        }
        closedList.insert(std::make_pair(olistData.xml_node, olistData.parent_xml_node));
    }

    if (!foundDest)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to compute path to destination.");
    }

    start_waypoint_following(&waypoints);
    publish_waypoints();

    RCLCPP_DEBUG(this->get_logger(), "Path computation successful.");
    response->routeid = 123456;
}

void
Global_Planner_OSM::publish_map_data()
{
    publish_marker_array(&map_data, publisher_map_data_);
}

void
Global_Planner_OSM::publish_waypoints()
{   
    RCLCPP_DEBUG(this->get_logger(), "Publish path on topic 'waypoints'");
    publish_marker_array(&waypoints, publisher_waypoints_, 1.0, 1.0, 0.0);
}

void
Global_Planner_OSM::publish_marker_array(
    std::vector<POSE> * data,
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> publisher,
    float size, float r, float g, float b, float a)
{
    std::vector<POSE>::iterator it;
    visualization_msgs::msg::MarkerArray marker_array;

    //map_data.push_back(std::make_pair(1.23456, 2.34567));

    int i = 0;
    for (it = data->begin(); it != data->end(); ++it)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "mapdata";
        marker.id = i++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        auto p = *it;

        marker.pose.position.x = (p.lon - 10.021944) * 110623.2362476;
        marker.pose.position.y = (p.lat - 53.555833) * 109632.4399804;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;
        
        marker.lifetime = rclcpp::Duration(0);

        marker_array.markers.push_back(marker);
    }

    publisher->publish(marker_array);
}

void
Global_Planner_OSM::clear_marker_array(std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> publisher)
{
    RCLCPP_DEBUG(this->get_logger(), "Clear all markers.");

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "mapdata";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    
    marker.lifetime = rclcpp::Duration(0);

    marker_array.markers.push_back(marker);
    publisher->publish(marker_array);
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

        ps.header.stamp = rclcpp::Clock().now();
        ps.header.frame_id = "0";
        ps.pose.position.x = (p.lon - 10.021944) * 110623.2362476;
        ps.pose.position.y = (p.lat - 53.555833) * 109632.4399804;
        ps.pose.position.z = 0.0;
        ps.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(p.angle);

        poses.push_back(ps);
    }

    waypoint_follower_goal_.poses = poses;
    
    RCLCPP_INFO(this->get_logger(), "Sending a path of %zu waypoints:", waypoint_follower_goal_.poses.size());
    for (auto waypoint : waypoint_follower_goal_.poses)
    {
        RCLCPP_DEBUG(this->get_logger(), "\t(%lf, %lf)", waypoint.pose.position.x, waypoint.pose.position.y);
    }

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

    //QBasicTimer timer.start(200, this);
}

void
Global_Planner_OSM::parseXml()
{   
    RCLCPP_DEBUG(this->get_logger(), "Parse xml");
    std::ifstream t(map_file_);

    buffer = std::vector<char>((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    buffer.push_back('\0');
    osm.parse<0>(&buffer[0]);

    root = osm.first_node("nodelist");
    map_data.clear();

    POSE p;
    for (rapidxml::xml_node<> *node = root->first_node("node"); node; node = node->next_sibling())
    {
        p.lat = strtod(node->first_attribute("lat")->value(), NULL);
        p.lon = strtod(node->first_attribute("lon")->value(), NULL);
        p.angle = 0.0;      // angle is not used

        map_data.push_back(p);
    }

    RCLCPP_INFO(this->get_logger(), "Parsed nodelist with id = %s", root->first_attribute(0)->value());
}

bool
Global_Planner_OSM::isDestination(rapidxml::xml_node<char> *pos, rapidxml::xml_node<char> *dest)
{
    RCLCPP_DEBUG(this->get_logger(),"Check if node with id %s equals desination node (id: %s).",
        pos->first_attribute("id")->value(),
        dest->first_attribute("id")->value());
    return (pos->first_attribute("id")->value() == dest->first_attribute("id")->value());
}

rapidxml::xml_node<char>
*Global_Planner_OSM::get_id(double const lat, double const lon)
{   
    rapidxml::xml_node<char> *nnode = nullptr;
    double lastDist = 10000.0;

    for (rapidxml::xml_node<> *node = root->first_node("node"); node; node = node->next_sibling())
    {
        double lat_ = strtod(node->first_attribute("lat")->value(), NULL);
        double lon_ = strtod(node->first_attribute("lon")->value(), NULL);

        double lastDist_;
        if((lastDist_ = std::sqrt(std::pow(lat_ - lat, 2) + std::pow(lon_ - lon, 2))) < lastDist )
        {
            lastDist = lastDist_;
            
            nnode = node;
        }
    }

    if (lastDist == 10000.0)
    {
        RCLCPP_INFO(this->get_logger(), "Could not find node at position %.7f, %.7f", lat, lon);
        return nullptr;
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Nearest Node to position %.7f, %.7f is node with id: %s\n",
                lat, lon, nnode->first_attribute("id")->value());
    }
    return nnode;
}

double
Global_Planner_OSM::distance_node_to_node(rapidxml::xml_node<char> *start, rapidxml::xml_node<char> *dest)
{
    double start_lat = strtod(start->first_attribute("lat")->value(), NULL);
    double start_lon = strtod(start->first_attribute("lon")->value(), NULL);
    double dest_lat = strtod(dest->first_attribute("lat")->value(), NULL);
    double dest_lon = strtod(dest->first_attribute("lon")->value(), NULL);

    return std::sqrt( std::pow(start_lat - dest_lat, 2.0) + std::pow(start_lon - dest_lon, 2.0) );
}

double
Global_Planner_OSM::cost_node_to_node(rapidxml::xml_node<char> *start, rapidxml::xml_node<char> *dest)
{
    // TODO: implement cost function
    return distance_node_to_node(start, dest);
}

void
Global_Planner_OSM::trace_path(std::unordered_map<rapidxml::xml_node<char> *, rapidxml::xml_node<char> *> closedList,
        rapidxml::xml_node<char> * dest)
{
    if (!waypoints.empty()) {waypoints.clear();}

    rapidxml::xml_node<char> *pid, *id;
    std::stack<rapidxml::xml_node<char> *> path;
    pid = dest;

    while (pid != NULL)
    {
        id = pid;
        path.push(id);
        pid = closedList.at(id);
    }

    RCLCPP_INFO(this->get_logger(), "Computed path:");

    double lat_, lon_;
    POSE pose;
    rapidxml::xml_node<char> *nextnode = path.top();
    pose.lat = strtod(nextnode->first_attribute("lat")->value(), NULL);
    pose.lon = strtod(nextnode->first_attribute("lon")->value(), NULL);
    path.pop();

    while (!path.empty())
    {   
        rapidxml::xml_node<char> *p = path.top();
        lat_ = strtod(p->first_attribute("lat")->value(), NULL);
        lon_ = strtod(p->first_attribute("lon")->value(), NULL);
        path.pop();
        // Calculate angle from nextnode to p
        // tan = G/A = y/x = lat/lon
        // tan 

        if (pose.lon == 0 && pose.lat == 0 )
        {
            pose.angle = 0.0;   // undefined
        } else {
            pose.angle = atan2(lat_ - pose.lat, lon_ - pose.lon);
        }

        RCLCPP_INFO(this->get_logger(), "Waypoint id: %s, lat: %.8f, lon: %.8f, angle: %f.",
               nextnode->first_attribute("id")->value(), pose.lat, pose.lon, pose.angle);
        waypoints.push_back(pose);

        pose.lat = lat_;
        pose.lon = lon_;
    }

    waypoints.push_back(pose);

}

}   // namespace nav_sgd

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<nav_sgd::Global_Planner_OSM>();

    //rclcpp::Service<sgd_msgs::srv::ComputePath>::SharedPtr service = 
    //    node->create_service<sgd_msgs::srv::ComputePath>("compute_path", &nav_sgd::Global_Planner_OSM::computePath);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to compute path");

    rclcpp::spin(node);
    rclcpp::shutdown();
}

