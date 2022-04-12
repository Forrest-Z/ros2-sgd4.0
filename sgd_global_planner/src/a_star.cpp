#include "sgd_global_planner/a_star.hpp"

namespace nav_sgd
{
    A_Star::A_Star(std::string osm_map_file, std::shared_ptr<A_Star_Users> users)
    {
        users_ = users;
        map_file_ = osm_map_file;
        
        // read nav file and try to find start and destination node
        doc.LoadFile(map_file_.c_str());

        // check error
        if (doc.ErrorID())
        {
            std::cerr << doc.ErrorStr();
            return;
        }

        if (doc.RootElement() == NULL)
        {
            std::cerr << "Error reading map file\n";
            return;
        }
    }

    A_Star::~A_Star()
    {
        // destructor
    }

    void
    A_Star::set_start(sgd_util::LatLon start)
    {
        start_node_ = node_near_lat_lon(start);
    }

    void 
    A_Star::set_dest(sgd_util::LatLon dest)
    {
        dest_node_ = node_near_lat_lon(dest);
    }

    void
    A_Star::set_dest(int node_id)
    {
        dest_node_ = node_with_id(node_id);
    }

    ROUTE
    A_Star::compute_path(std::string username)
    {
        // set user
        users_->set_user(username);

        // define comparator for openlist and initialize closed list
        auto compare = [](A_Star_Node a, A_Star_Node b) { return a.f() > b.f(); };
        std::priority_queue<A_Star_Node, std::vector<A_Star_Node>, decltype(compare)> openList(compare);
        std::unordered_map<long, A_Star_Node> closedList;

        // push start node to openList
        openList.push(start_node_);

        // initially the destination is not reached
        while (!openList.empty())
        {
            // get node with lowest f score from openlist and remove node afterwards
            A_Star_Node current_node = openList.top();
            openList.pop();
            if (closedList.count(current_node.id()) > 0)   continue;

            // if current node is destination, terminate algorithm and reconstruct path
            if (current_node == dest_node_)
            {
                closedList.insert({current_node.id(), current_node});
                return trace_path(closedList, current_node);
            }

            // Go through xml document and get all neighbour nodes
            tinyxml2::XMLElement *next_node = doc.RootElement()->FirstChildElement("node");
            while (next_node != NULL)
            {
                long next_id = strtol(next_node->Attribute("id"), NULL, 10);
                if (current_node.has_neighbor(next_id) && closedList.count(next_id) == 0)
                {
                    A_Star_Node n(next_node, current_node, users_);
                    // set heuristic
                    n.set_h(n.get_latlon().distance_to(dest_node_.get_latlon()));
                    openList.push(n);
                }
                next_node = next_node->NextSiblingElement();
            }
            
            closedList.insert({current_node.id(), current_node});
        }
        std::cout << "Failed to compute path to destination.\n";

        ROUTE route;
        route.length_m = -1;
        return route;
    }

    A_Star_Node
    A_Star::node_near_lat_lon(sgd_util::LatLon latlon)
    {
        tinyxml2::XMLElement *nnode = nullptr;
        double lastDist = 10000.1;

        tinyxml2::XMLElement *node = doc.RootElement()->FirstChildElement("node");
        while (node != NULL)
        {
            double lat_ = strtod(node->Attribute("lat"), NULL);
            double lon_ = strtod(node->Attribute("lon"), NULL);

            double lastDist_ = latlon.distance_to(lat_, lon_);
            if (lastDist_ < lastDist)
            {
                lastDist = lastDist_;
                nnode = node;
            }

            node = node->NextSiblingElement();
        }

        if (lastDist >= 10000.0)
        {
            throw std::logic_error("Could not find node near " + latlon.to_string());
        }
        return A_Star_Node(nnode, users_);
    }

    A_Star_Node
    A_Star::node_with_id(int node_id)
    {
        tinyxml2::XMLElement *node = doc.RootElement()->FirstChildElement("node");

        while (node != NULL)
        {
            double id_ = strtod(node->Attribute("id"), NULL);
            if (id_ == node_id)
            {
                return A_Star_Node(node, users_);
            }

            node = node->NextSiblingElement();
        }
        throw std::logic_error("Could not find node with id " + std::to_string(node_id));
    }

    ROUTE
    A_Star::trace_path(std::unordered_map<long, A_Star_Node> closedList, A_Star_Node dest)
    {
        std::vector<sgd_util::LatLon> waypoints;
        std::stack<A_Star_Node> path;

        long id = dest.id();
        path.push(dest);

        while (id > 0)
        {
            path.push(closedList.at(id));
            id = closedList.at(id).predecessor_id();
        }

        double length_m = 0.0;
        auto last_ll = path.top().get_latlon();
        while (!path.empty())
        {
            auto ll = path.top().get_latlon();
            length_m += last_ll.distance_to(ll);
            waypoints.push_back(ll);

            last_ll = ll;
            path.pop();
        }

        // add values to route
        ROUTE route;
        route.cost = dest.f();
        route.length_m = length_m;
        route.waypoints = waypoints;
        return route;
    }

} // namespace nav_sgd