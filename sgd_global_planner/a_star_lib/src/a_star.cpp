//#include "a_star.hpp"
#include "../include/a_star.hpp"

namespace sgd_global_planner
{

A_Star::A_Star(std::string osm_map_file, std::shared_ptr<A_Star_Users> users)
{
    PLOGD << "Initializing...";
    users_ = users;
    users->set_user("default");
    
    // load map
    load_map(osm_map_file);
}

A_Star::~A_Star() {}

void
A_Star::load_map(std::string nav_filename)
{
    PLOGD.printf("Load map file: %s", nav_filename.c_str());
    nodelist_.clear();

    tinyxml2::XMLDocument doc;
    doc.LoadFile(nav_filename.c_str());

    // check if errors occured
    if (doc.ErrorID())
    {
        PLOGE.printf("Error reading file %s: %s", nav_filename.c_str(), doc.ErrorStr());
        return;
    }

    if (doc.RootElement() == NULL)
    {
        PLOGE.printf("Error reading map file %s: No root element found.", nav_filename.c_str());
        return;
    }

    PLOGD << "Reading nodes...";
    // read xml file and load all nodes to unordered map
    tinyxml2::XMLElement *node = doc.RootElement()->FirstChildElement("node");
    while (node != NULL)
    {
        // get id from attribute
        auto id = strtoll(node->Attribute("id"), NULL, 10);

        // create new node with the read id and position
        A_Star_Node n(id, sgd_util::LatLon(strtod(node->Attribute("lat"), NULL), strtod(node->Attribute("lon"), NULL)));
        if (node->Attribute("pid") != NULL)
        {
            n.set_pid(strtoll(node->Attribute("pid"), NULL, 10));
        }
        // insert into nodelist
        //PLOGD.printf("%lld", id);
        nodelist_.insert({id, n});

        node = node->NextSiblingElement();
    }

    PLOGD << "Add neighbors to nodes...";
    // go through xml again and add neighbours and paths to nodes
    node = doc.RootElement()->FirstChildElement("node");
    while (node != NULL)
    {
        // get id from attribute
        auto id = strtoll(node->Attribute("id"), NULL, 10);

        // get first neighbor node
        tinyxml2::XMLElement *nd1 = node->FirstChildElement("nd");
        while (nd1 != NULL)
        {
            // get id from neighbor node
            auto ref_id1 = strtoll(nd1->Attribute("ref"), NULL, 10);
            // check if neighbor node is contained in nodelist
            if (nodelist_.count(ref_id1) > 0)
            {
                // get tags (e.g. width, surface, ...)
                auto tag = nd1->FirstChildElement();
                // create new path to neighbor node
                A_Star_Path path(ref_id1);
                while (tag)
                {
                    // add tags to path
                    path.add_attribute(tag->Value(), tag->GetText());
                    //f *= users_->calculate_factor(tag->Value(), tag->GetText());
                    tag = tag->NextSiblingElement();
                }

                // add path to neighbor to node
                nodelist_.at(id).add_neighbour(path);
            }
            else
            {
                PLOGW.printf("Node %lld is specified as reference but could not be found in nodelist.", ref_id1);
            }

            nd1 = nd1->NextSiblingElement();
        }
        // A_Star_Node n(id, sgd_util::LatLon(strtod(node->Attribute("lat"), NULL), strtod(node->Attribute("lon"), NULL)));
        // nodelist_.insert({id, n});

        node = node->NextSiblingElement();
    }
    PLOGI.printf("Successfully loaded %d nodes from map file!", nodelist_.size());
}

void
A_Star::set_user(std::string username)
{
    // set user
    PLOGD.printf("Set username to %s", username.c_str());
    users_->set_user(username);
}

ROUTE
A_Star::compute_path(sgd_util::LatLon start, sgd_util::LatLon dest)
{
    return compute_path(start, node_near_lat_lon(dest));
}

ROUTE
A_Star::compute_path(sgd_util::LatLon start, llong dest_node_id)
{
    // reset A*
    closedList.clear();
    for (auto n : nodelist_)
    {
        n.second.reset();   
    }

    llong start_node_id = node_near_lat_lon(start);

    PLOGD.printf("Compute path to node %lld", dest_node_id);
    if (nodelist_.count(start_node_id) < 1 || nodelist_.count(dest_node_id) < 1)
    {
        throw std::invalid_argument("Start or destination node not set!");
    }

    // operation; curr_node; next_node; pred_bearing; next_bearing; result_bearing; ang_factor;
                //        user factor along path; pred_cost; distance to dest
    PLOGD << "Operation; node id; pred_bearing; next_bearing; result_bearing; ang_factor; "
            << "user factor along path; pred_cost (g); distance to dest (h); cost (f)";

    auto t_beg = std::chrono::system_clock::now();
    // define comparator for openlist and initialize closed list
    auto compare = [](A_Star_Node a, A_Star_Node b) { return a.f() > b.f(); };
    std::priority_queue<A_Star_Node, std::vector<A_Star_Node>, decltype(compare)> openList(compare);
    
    // push start node to openList
    PLOGD.printf("Push to openlist: %lld", nodelist_.at(start_node_id).id());
    openList.push(nodelist_.at(start_node_id));
    auto dest_ll = nodelist_.at(dest_node_id).get_latlon();   // position of destination node

    // initially the destination is not reached
    while (!openList.empty())
    {
        // get node with lowest f score from openlist and remove node afterwards
        A_Star_Node current_node(openList.top());
        PLOGD.printf("Expand node; %lld; %lld; %.3f", current_node.id(), current_node.predecessor_id(), current_node.f());
        openList.pop();

        // if current node is already on the closed list, continue
        if (closedList.count(current_node.id()) > 0)   continue;

        // if current node is destination, terminate algorithm and reconstruct path
        if (current_node.id() == dest_node_id)
        {
            PLOGD << "Found destination node.";
            closedList.insert({current_node.id(), current_node});

            auto t_end = std::chrono::system_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_beg);
            PLOGD.printf("Path computation took %lldms", duration.count());
            return trace_path(closedList, current_node);
        }

        // get all paths from current node
        // add destination nodes to openList
        auto pred_bearing_ = current_node.get_predecessor_bearing();
        for (auto p : current_node.paths())
        {
            // get destination id
            auto path_dest_id_ = p.get_dest();
            // get destination node
            A_Star_Node path_dest_(nodelist_.at(path_dest_id_));
            // check if node is already on closed list or if node is blocked
            if (closedList.count(path_dest_id_) < 1 && !path_dest_.is_blocked())
            {
                double angle_factor = 1.0, next_bearing_ = 0.0, result_bearing_ = 0.0;
                double side_factor_ = 1.0;
                double base_bearing = 0.0;

                next_bearing_ = current_node.get_latlon().bearing(path_dest_.get_latlon());

                // get parent node -> bearing from parent node to next parent node
                auto parentnode = nodelist_.at(current_node.pid());
                auto dest_pid = nodelist_.at(path_dest_.pid());
                base_bearing = (parentnode == dest_pid) ? next_bearing_ : parentnode.get_latlon().bearing(dest_pid.get_latlon());

                // check if we are on the right side or left side of the way -> move to JOSM plugin
                if (path_dest_id_ != path_dest_.pid() && current_node.id() != current_node.pid())
                {
                    // calculate angle from current node to next parent node
                    double bear_id_pid = current_node.get_latlon().bearing(dest_pid.get_latlon());
                    auto bear = (base_bearing - bear_id_pid)
                                + (base_bearing - bear_id_pid < -M_PI) * 2*M_PI 
                                - (base_bearing - bear_id_pid >  M_PI) * 2*M_PI;
                    side_factor_ = (bear > 0) ? 0.85 : 1.15;
                }

                // set angle to interval [-PI, PI]
                result_bearing_ = (base_bearing - next_bearing_)
                                + (base_bearing - next_bearing_ < -M_PI) * 2*M_PI 
                                - (base_bearing - next_bearing_ >  M_PI) * 2*M_PI;
                angle_factor = users_->angle_factor(result_bearing_);

                // set distance to destination as heuristic
                path_dest_.set_h(path_dest_.get_latlon().distance_to(dest_ll));
                // set current node as predecessor for next node
                auto g = path_dest_.set_predecessor(current_node, p.cost(users_)*side_factor_, angle_factor);    // angle_factor

                PLOGD.printf("Push to openList; %lld; %.3f; %.3f; %.3f; %.3f; %.3f; %.3f; %.3f; %.3f",
                        path_dest_id_, pred_bearing_, next_bearing_, result_bearing_, angle_factor,
                        p.cost(users_), g, path_dest_.get_latlon().distance_to(dest_ll), path_dest_.f());
                // push next node to openlist
                openList.push(path_dest_);
            }
        }
        
        PLOGD.printf("Add to closedList; %lld", current_node.id());
        closedList.insert({current_node.id(), current_node});
    }
    PLOGW << "Failed to compute path to destination.";

    ROUTE route;
    route.length_m = -1;

    auto t_end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_beg);
    PLOGD.printf("Path computation failed after %lldms", duration.count());

    return route;
}

llong
A_Star::node_near_lat_lon(sgd_util::LatLon latlon)
{
    PLOGD.printf("Try to find node near %s", latlon.to_string().c_str());
    double lastDist = 10000.1;
    llong id_;
    for (auto nd : nodelist_)
    {
        double lastDist_ = latlon.distance_to(nd.second.get_latlon());
        if (lastDist_ < lastDist)
        {
            lastDist = lastDist_;
            PLOGV.printf("Found node %lld (%s) at %.3fm distance", nd.first, nd.second.get_latlon().to_string().c_str(), lastDist);
            id_ = nd.first;
        }
    }
    if (lastDist >= 100.0)  // maximum distance to nearest node
    {
        throw std::logic_error("Could not find node near " + latlon.to_string());
    }
    PLOGD.printf("Found node %lld at %.3fm distance", id_, lastDist);
    return id_;
}

ROUTE
A_Star::trace_path(std::unordered_map<llong, A_Star_Node> closedList, A_Star_Node dest)
{
    std::vector<sgd_util::LatLon> waypoints;
    std::stack<sgd_util::LatLon> path;

    llong id = dest.id();
    // PLOGD.printf("Push node at position %s to path", dest.get_latlon().to_string().c_str());
    //path.push(dest.get_latlon());

    while (id > 0)
    {
        //PLOGD.printf("Push node at position %s to path", closedList.at(id).get_latlon().to_string().c_str());
        path.push(closedList.at(id).get_latlon());
        id = closedList.at(id).predecessor_id();
    }

    double length_m = 0.0;
    auto last_ll = path.top();
    while (!path.empty())
    {
        auto ll = path.top();
        length_m += last_ll.distance_to(ll);
        waypoints.push_back(ll);

        last_ll = ll;
        path.pop();
    }

    PLOGD.printf("Return path with %d waypoints and a length of %.3fm", waypoints.size(), length_m);

    // add values to route
    ROUTE route;
    route.cost = dest.f();
    route.length_m = length_m;
    route.waypoints = waypoints;
    return route;
}

std::vector<A_Star_Node>
A_Star::get_nodelist()
{
    // copy nodes to vector
    std::vector<A_Star_Node> nodes_;
    for (auto entry : nodelist_)
    {
        nodes_.push_back(entry.second);
    }
    return nodes_;
}

} // namespace nav_sgd