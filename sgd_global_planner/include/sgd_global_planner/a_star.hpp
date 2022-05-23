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
#include <ctime>
#include <chrono>

#include <stack>
#include <map>
#include <unordered_map>
#include <queue>

#include "tinyxml2.h"
#include "a_star_users.hpp"
#include "a_star_node.hpp"
#include "sgd_util/geotools.hpp"

namespace nav_sgd
{

typedef int64_t llong;  // format used to store the ids

struct ROUTE
{
    double cost;
    double length_m;
    std::vector<sgd_util::LatLon> waypoints;
};

class A_Star
{

public:
    A_Star(std::string osm_map_file, std::shared_ptr<A_Star_Users> users, std::string log_dir);
    ~A_Star();

    void load_map(std::string nav_filename);

    void set_start(sgd_util::LatLon start);
    void set_dest(sgd_util::LatLon dest);
    void set_dest(llong node_id);
    void set_username(std::string username);

    /**
     * @brief Compute the optimal path from a starting point to a destination point.
     * 
     * @return true if path computation was succesful, false otherwise
     */
    ROUTE compute_path();

private:
    //std::string map_file_;
    std::unordered_map<llong, A_Star_Node> nodelist_;

    //std::string adr_file;
    std::shared_ptr<A_Star_Users> users_ = nullptr;

    std::ofstream log_file;

    // start and destination nodes
    llong start_node_, dest_node_;
    std::unordered_map<llong, A_Star_Node> closedList;

    /**
     * @brief Find the nearest node to the specified lat/lon
     * 
     * @param latlon 
     * @return node id
     */
    llong node_near_lat_lon(sgd_util::LatLon latlon);
    //A_Star_Node node_from_xml(tinyxml2::XMLElement *node);

    /**
     * @brief Traces the path and outputs all nodes in the correct order
     * 
     * @param closedList computed nodes from a star
     * @param dest destination node
     */
    ROUTE trace_path(std::unordered_map<llong, A_Star_Node> closedList, A_Star_Node dest);

    inline void log(std::string msg)
    {
        auto now = std::chrono::system_clock::now().time_since_epoch();
        log_file << "[a_star] [" << now.count() << "]: " << msg << std::endl;
    }

};

}   // namespace nav_sgd

#endif