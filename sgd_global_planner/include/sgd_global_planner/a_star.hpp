#ifndef NAV_SGD_A_STAR_HPP_
#define NAV_SGD_A_STAR_HPP_

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

#include "tinyxml2.h"
#include "a_star_users.hpp"

namespace nav_sgd
{

class A_Star
{

//! \brief Struct to hold node id, latitude and longitude
struct NODE
{
    tinyxml2::XMLElement * xml_node;
    tinyxml2::XMLElement * parent_xml_node;
    double f,g,h;
};

//! \brief Struct to hold position information latitude, longitude and angle around z in radians.
struct POSE
{
    double lat, lon, angle;
};

private:
    const std::string map_file_;
    // Store waypoints from start to destination
    std::vector<POSE> waypoints;
    // Store map data
    std::vector<POSE> map_data;
    A_Star_Users* users;

    //! \brief Read and parse xml file
    //! \returns Root node of xml document
    tinyxml2::XMLElement * read_nav_file();

    tinyxml2::XMLElement * get_node_from_lat_lon(tinyxml2::XMLElement * root, double const lat, double const lon);
    void trace_path(std::unordered_map<tinyxml2::XMLElement *, tinyxml2::XMLElement *> closedList, tinyxml2::XMLElement * dest);
    double calc_cost_factor(tinyxml2::XMLElement *node);
    double distance_node_to_node(tinyxml2::XMLElement *start, tinyxml2::XMLElement *dest);
    bool isDestination(tinyxml2::XMLElement *pos, tinyxml2::XMLElement *dest);
    
public:
    A_Star(std::string osm_map_file, std::string users_file);
    ~A_Star();

    int compute_path(double start_lat, double start_lon, double end_lat, double end_lon, std::string username="default");
};

A_Star::A_Star(std::string osm_map_file, std::string users_file)
    : map_file_(osm_map_file)
{
    users = new A_Star_Users(users_file);
}

A_Star::~A_Star()
{
    // destrcutor
}

int
A_Star::compute_path(double start_lat, double start_lon, double end_lat, double end_lon, std::string username)
{
    // read nav file and try to find start and destination node
    auto root = read_nav_file();
    auto start_node = get_node_from_lat_lon(root, start_lat, start_lon);
    auto dest_node = get_node_from_lat_lon(root, end_lat, end_lon);

    // set user
    users->set_user(username);

    // create open list and closed list
    auto compare = [](NODE a, NODE b) {return a.f > b.f;};
    std::priority_queue<NODE, std::vector<NODE>, decltype(compare)> openList(compare);
    std::unordered_map<tinyxml2::XMLElement *, tinyxml2::XMLElement *> closedList;

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

        tinyxml2::XMLElement * pos = olistData.xml_node;

        // Remove this node from the open list
        openList.pop();

        if (isDestination(pos, dest_node))
        {
            closedList.insert(std::make_pair(olistData.xml_node, olistData.parent_xml_node));
            foundDest = true;
            trace_path(closedList, dest_node);

            break;
        }

        tinyxml2::XMLElement * nd = pos->FirstChildElement("nd");
        while (nd)
        {
            long pos_id = strtol(nd->Attribute("ref"), NULL, 10);

            tinyxml2::XMLElement * np = root->FirstChildElement("node");
            while (np)
            {
                if (strtol(np->Attribute("id"), NULL, 10) == pos_id)
                {
                    // If successor is already on the closed list ignore it.
                    if (closedList.find(np) != closedList.end()) continue;

                    NODE n;
                    n.xml_node = np;
                    n.parent_xml_node = olistData.xml_node;
                    n.g = olistData.g + distance_node_to_node(np, dest_node) * calc_cost_factor(nd);
                    n.h = distance_node_to_node(np, dest_node);
                    n.f = n.g + n.h;

                    openList.push(n);

                    break;
                }

                np = np->NextSiblingElement();
            }

            nd = nd->NextSiblingElement();
        }
        closedList.insert(std::make_pair(olistData.xml_node, olistData.parent_xml_node));
    }

    if (!foundDest)
    {
        std::cout << "Failed to compute path to destination.\n";
    }

    // TODO write nodes to file
    



    //start_waypoint_following(&waypoints);
    //publish_waypoints();

    //response->routeid = 123456;
}

tinyxml2::XMLElement *
A_Star::read_nav_file()
{   
    tinyxml2::XMLDocument doc;
    doc.LoadFile(map_file_.c_str());

    // check error
    if (doc.ErrorID())
    {
        std::cerr << doc.ErrorStr();
        return;
    }

    return doc.RootElement();
}

tinyxml2::XMLElement *
A_Star::get_node_from_lat_lon(tinyxml2::XMLElement * root, const double lat, const double lon)
{
    tinyxml2::XMLElement * nnode = nullptr;
    double lastDist = 10000.0;

    tinyxml2::XMLElement * node = root->FirstChildElement("node");
    while (node)
    {
        double lat_ = strtod(node->Attribute("lat"), NULL);
        double lon_ = strtod(node->Attribute("lon"), NULL);

        double lastDist_;
        if((lastDist_ = std::sqrt(std::pow(lat_ - lat, 2) + std::pow(lon_ - lon, 2))) < lastDist )
        {
            lastDist = lastDist_;
            
            nnode = node;
        }

        node = node->NextSiblingElement();
    }

    if (lastDist == 10000.0)
    {
        //RCLCPP_INFO(this->get_logger(), "Could not find node at position %.7f, %.7f", lat, lon);
        return nullptr;
    }
    return nnode;
}

bool
A_Star::isDestination(tinyxml2::XMLElement * pos, tinyxml2::XMLElement * dest)
{
    //RCLCPP_DEBUG(this->get_logger(),"Check if node with id %s equals desination node (id: %s).",
    //    pos->first_attribute("id")->value(),
    //    dest->first_attribute("id")->value());
    return (pos->Attribute("id") == dest->Attribute("id"));
}

double
A_Star::distance_node_to_node(tinyxml2::XMLElement * start, tinyxml2::XMLElement * dest)
{
    double start_lat = strtod(start->Attribute("lat"), NULL);
    double start_lon = strtod(start->Attribute("lon"), NULL);
    double dest_lat = strtod(dest->Attribute("lat"), NULL);
    double dest_lon = strtod(dest->Attribute("lon"), NULL);

    return std::sqrt( std::pow(start_lat - dest_lat, 2.0) + std::pow(start_lon - dest_lon, 2.0) );
}

double
A_Star::calc_cost_factor(tinyxml2::XMLElement * node)
{
    // gehe durch alle child nodes 
    // -> get name and value
    double f = 0;
    tinyxml2::XMLElement * n = node->FirstChildElement("node");
    while (n)
    {
        f += users->calculate_factor(n->Value(), n->ToText()->Value());
        n = n->NextSiblingElement();
    }
    return f <= 0 ? 1E6 : f;
}

void
A_Star::trace_path(std::unordered_map<tinyxml2::XMLElement *, tinyxml2::XMLElement *> closedList,
        tinyxml2::XMLElement * dest)
{
    if (!waypoints.empty()) waypoints.clear();

    tinyxml2::XMLElement *pid, *id;
    std::stack<tinyxml2::XMLElement *> path;
    pid = dest;

    while (pid != NULL)
    {
        id = pid;
        path.push(id);
        pid = closedList.at(id);
    }

    //RCLCPP_INFO(this->get_logger(), "Computed path:");

    double lat_, lon_;
    POSE pose;
    tinyxml2::XMLElement * nextnode = path.top();
    pose.lat = strtod(nextnode->Attribute("lat"), NULL);
    pose.lon = strtod(nextnode->Attribute("lon"), NULL);
    path.pop();

    while (!path.empty())
    {   
        tinyxml2::XMLElement * p = path.top();
        lat_ = strtod(p->Attribute("lat"), NULL);
        lon_ = strtod(p->Attribute("lon"), NULL);
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

        //RCLCPP_INFO(this->get_logger(), "Added waypoint with lat: %.8f, lon: %.8f, angle: %f.",
        //       pose.lat, pose.lon, pose.angle);
        waypoints.push_back(pose);
        std::cout << "Waypoint lat: " << pose.lat << ", lon: " << pose.lon << std::endl;

        pose.lat = lat_;
        pose.lon = lon_;
    }

    waypoints.push_back(pose);
}

}   // namespace nav_sgd

#endif