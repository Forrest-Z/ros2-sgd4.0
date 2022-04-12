// Copyright 2022 HAW Hamburg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV_SGD_A_STAR_HPP_
#define NAV_SGD_A_STAR_HPP_

#include <string>
#include <memory>
#include <vector>
#include <fstream>
#include <iostream>

#include <stack>
#include <unordered_map>
#include <queue>

#include "tinyxml2.h"
#include "a_star_users.hpp"
#include "a_star_node.hpp"
#include "sgd_util/geotools.hpp"

namespace nav_sgd
{

struct ROUTE
{
    double cost;
    double length_m;
    std::vector<sgd_util::LatLon> waypoints;
};

class A_Star
{

public:
    A_Star(std::string osm_map_file, std::shared_ptr<A_Star_Users> users);
    ~A_Star();

    void set_start(sgd_util::LatLon start);
    void set_dest(sgd_util::LatLon dest);
    void set_dest(int node_id);

    /**
     * @brief Compute the optimal path from a starting point to a destination point.
     * 
     * @param username Username or "default" if not defined
     * @return true if path computation was succesful, false otherwise
     */
    ROUTE compute_path(std::string username="default");

private:
    std::string map_file_;
    std::string adr_file;
    std::shared_ptr<A_Star_Users> users_ = nullptr;
    tinyxml2::XMLDocument doc;

    // start and destination nodes
    A_Star_Node start_node_, dest_node_;

    /**
     * @brief Find the nearest node to the specified lat/lon
     * 
     * @param latlon 
     * @return A_Star_Node 
     */
    A_Star_Node node_near_lat_lon(sgd_util::LatLon latlon);
    A_Star_Node node_with_id(int node_id);

    /**
     * @brief Traces the path and outputs all nodes in the correct order
     * 
     * @param closedList computed nodes from a star
     * @param dest destination node
     */
    ROUTE trace_path(std::unordered_map<long, A_Star_Node> closedList, A_Star_Node dest);

};

}   // namespace nav_sgd

#endif