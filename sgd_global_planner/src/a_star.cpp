#include "sgd_global_planner/a_star.hpp"

namespace nav_sgd
{
    A_Star::A_Star(std::string osm_map_file, std::shared_ptr<A_Star_Users> users, std::string log_dir)
    {
        users_ = users;
        users->set_user("default");

        // Write log header file
        time_t now = time(0);
        tm *ltm = localtime(&now);

        const char *out = "a-star_%d-%d-%d_%d-%d-%d";
        int sz = std::snprintf(nullptr, 0, out, 1900+ltm->tm_year, 1+ltm->tm_mon, ltm->tm_mday,
                                ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
        char buf[sz + 1]; // note +1 for null terminator
        std::snprintf(&buf[0], sz+1, out, 1900+ltm->tm_year, 1+ltm->tm_mon, ltm->tm_mday,
                                ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
        
        log_file.open(log_dir + std::string(buf) + ".log");
        
        // load map
        load_map(osm_map_file);
    }

    A_Star::~A_Star()
    {
        // destructor
        log_file.close();
    }

    void
    A_Star::load_map(std::string nav_filename)
    {
        log("Load map file: " + nav_filename);
        nodelist_.clear();

        tinyxml2::XMLDocument doc;
        doc.LoadFile(nav_filename.c_str());

        // check if errors occured
        if (doc.ErrorID())
        {
            std::cerr << doc.ErrorStr();
            log(doc.ErrorStr());
            return;
        }

        if (doc.RootElement() == NULL)
        {
            std::cerr << "Error reading map file\n";
            log("Error reading map file.");
            return;
        }

        // load all nodes to unordered map
        tinyxml2::XMLElement *node = doc.RootElement()->FirstChildElement("node");
        while (node != NULL)
        {
            auto id = strtoll(node->Attribute("id"), NULL, 10);

            A_Star_Node n(id, sgd_util::LatLon(strtod(node->Attribute("lat"), NULL), strtod(node->Attribute("lon"), NULL)));
            nodelist_.insert({id, n});

            node = node->NextSiblingElement();
        }

        // add neighbours and paths to nodes
        node = doc.RootElement()->FirstChildElement("node");
        while (node != NULL)
        {
            auto id = strtoll(node->Attribute("id"), NULL, 10);
            //auto curr_node = &nodelist_.at(id);

            tinyxml2::XMLElement *nd1 = node->FirstChildElement("nd"); // get first neighbor node
            while (nd1)
            {
                auto ref_id1 = strtoll(nd1->Attribute("ref"), NULL, 10);
                if (nodelist_.count(ref_id1) > 0)    // && closedList.count(id) < 1
                {
                    auto ref_node = &nodelist_.at(ref_id1);
                    //double f = 1.0;
                    auto tag = nd1->FirstChildElement();
                    A_Star_Path path(ref_node);
                    while (tag)
                    {
                        path.add_attribute(tag->Value(), tag->GetText());
                        //f *= users_->calculate_factor(tag->Value(), tag->GetText());
                        tag = tag->NextSiblingElement();
                    }

                    nodelist_.at(id).add_neighbour(path);
                    //curr_node->add_neighbour(ref_node->id(), (f <= 0) ? 1E6 : f);
                }
                else
                {
                    log("Node " + std::to_string(ref_id1) + " is specified as reference but could not be found in nodelist.");
                }

                nd1 = nd1->NextSiblingElement();
            }
            A_Star_Node n(id, sgd_util::LatLon(strtod(node->Attribute("lat"), NULL), strtod(node->Attribute("lon"), NULL)));
            nodelist_.insert({id, n});

            node = node->NextSiblingElement();
        }
        log("Map file successfully loaded!");
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
    A_Star::set_dest(llong node_id)
    {
        if (nodelist_.count(node_id) > 0)
        {
            dest_node_ = node_id;
        }
        else
        {
            log("Node with id " + std::to_string(node_id) + " not found in nodelist.");
        }
    }

    void
    A_Star::set_username(std::string username)
    {
        // set user
        log("Set username to " + username);
        users_->set_user(username);
    }

    ROUTE
    A_Star::compute_path()
    {
        log("Compute path to node " + std::to_string(dest_node_));
        if (nodelist_.count(start_node_) < 1 || nodelist_.count(dest_node_) < 1)
        {
            throw std::invalid_argument("Start or destination node not set!");
        }

        auto t_beg = std::chrono::system_clock::now();
        // define comparator for openlist and initialize closed list
        auto compare = [](A_Star_Node *a, A_Star_Node *b) { return a->f() > b->f(); };
        std::priority_queue<A_Star_Node*, std::vector<A_Star_Node*>, decltype(compare)> openList(compare);
        closedList.clear();

        // push start node to openList
        log("Push start node to openlist: " + std::to_string(nodelist_.at(start_node_).id()));
        openList.push(&nodelist_.at(start_node_));
        auto dest_ll = nodelist_.at(dest_node_).get_latlon();   // position of destination node

        // initially the destination is not reached
        while (!openList.empty())
        {
            // get node with lowest f score from openlist and remove node afterwards
            auto current_node = openList.top();
            log("    --> Expand node " + std::to_string(current_node->id()));
            openList.pop();
            if (closedList.count(current_node->id()) > 0)   continue;

            // if current node is destination, terminate algorithm and reconstruct path
            if (current_node->id() == dest_node_)
            {
                log("Found destination node.");
                closedList.insert({current_node->id(), *current_node});

                auto t_end = std::chrono::system_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_beg);
                log("Path computation took " + std::to_string(duration.count()) + "ms");
                return trace_path(closedList, *current_node);
            }

            // get all paths from current node
            // add destination nodes to openList
            log("Go through paths from current node");
            auto ang1 = current_node->get_path_angle();
            for (auto p : current_node->paths())
            {
                auto nd = p.get_dest();
                if (closedList.count(nd->id()) < 1 && !nd->is_blocked())
                {
                    // node is not already on closed list
                    double angle_factor = 1.0;
                    if (current_node->id() != start_node_)
                    {
                        auto ang2 = current_node->get_latlon().bearing(nd->get_latlon());

                        // set angle to interval [-PI, PI]
                        double ang = (ang1 - ang2) + (ang1-ang2 < -M_PI) * 2*M_PI - (ang1-ang2 > M_PI) * 2*M_PI;
                        //angle_factor = users_->angle_factor(ang);

                        log("Angle factor - ang1: " + std::to_string(ang1) + ", ang2: " + std::to_string(ang2) + " = factor: " + std::to_string(ang));
                    }
                    
                    nd->set_h(nd->get_latlon().distance_to(dest_ll));
                    auto g = nd->set_predecessor(current_node, p.cost(users_), angle_factor);

                    log("Add " + std::to_string(nd->id()) + " to openList with base-cost " + std::to_string(p.cost(users_)) + " -> "
                            + std::to_string(g) + "; heuristic: " + std::to_string(nd->get_latlon().distance_to(dest_ll)));
                    openList.push(nd);
                }
            }
            
            log("Add " + std::to_string(current_node->id()) + " to closedList");
            closedList.insert({current_node->id(), *current_node});
        }
        log("Failed to compute path to destination.");

        ROUTE route;
        route.length_m = -1;

        auto t_end = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_beg);
        log("Path computation took " + std::to_string(duration.count()) + "ms");
        return route;
    }

    llong
    A_Star::node_near_lat_lon(sgd_util::LatLon latlon)
    {
        log("Try to find node near " + latlon.to_string());
        double lastDist = 10000.1;
        llong id_;
        for (auto nd : nodelist_)
        {
            double lastDist_ = latlon.distance_to(nd.second.get_latlon());
            if (lastDist_ < lastDist)
            {
                lastDist = lastDist_;
                log("Found node " + std::to_string(nd.first) + " (" + nd.second.get_latlon().to_string() + ") in " + std::to_string(lastDist) + "m distance.");
                id_ = nd.first;
            }
        }
        if (lastDist >= 100.0)  // maximum distance to nearest node
        {
            throw std::logic_error("Could not find node near " + latlon.to_string());
        }
        log("Found node " + std::to_string(id_) + " in " + std::to_string(lastDist) + "m distance.");
        return id_;
    }

    // A_Star_Node
    // A_Star::node_from_xml(tinyxml2::XMLElement *node)
    // {
    //     // get coordinates
    //     auto ll = sgd_util::LatLon(strtod(node->Attribute("lat"), NULL),
    //                                strtod(node->Attribute("lon"), NULL));

    //     // create new node
    //     auto a_star_node = A_Star_Node(strtoll(node->Attribute("id"), NULL, 10), ll);
    //     log("Get node from xml " + std::to_string(a_star_node.id()));

    //     // go through neighbours
    //     tinyxml2::XMLElement *nd = node->FirstChildElement("nd"); // get first neighbor node
    //     while (nd)
    //     {
    //         auto id = strtoll(nd->Attribute("ref"), NULL, 10);
    //         if (nd->FirstChildElement())    // && closedList.count(id) < 1
    //         {
    //             double f = 1.0;
    //             auto tag = nd->FirstChildElement();
    //             while (tag)
    //             {
    //                 f *= users_->calculate_factor(tag->Value(), tag->GetText());
    //                 tag = tag->NextSiblingElement();
    //             }

    //             log("Add neighbour " + std::to_string(id) + " with base-cost " + std::to_string((f <= 0) ? 1E6 : f));
    //             a_star_node.add_neighbour(id, (f <= 0) ? 1E6 : f);
    //         }

    //         nd = nd->NextSiblingElement();
    //     }
    //     return a_star_node;
    // }

    ROUTE
    A_Star::trace_path(std::unordered_map<llong, A_Star_Node> closedList, A_Star_Node dest)
    {
        std::vector<sgd_util::LatLon> waypoints;
        std::stack<A_Star_Node> path;

        llong id = dest.id();
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