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

#ifndef SGD_GLOBAL_PLANNER__A_STAR_HPP_
#define SGD_GLOBAL_PLANNER__A_STAR_HPP_

#include <string>
#include <memory>
#include <vector>
#include <ctime>
#include <chrono>

#include <stack>
#include <map>
#include <unordered_map>
#include <queue>

#include "tinyxml2.h"
#include "a_star_users.hpp"
#include "a_star_node.hpp"
#include "a_star_path.hpp"
#include "sgd_util/geotools.hpp"

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

namespace sgd_global_planner
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

    /**
     * @brief Load the map
     * 
     * @param nav_filename 
     */
    void load_map(std::string nav_filename);

    /**
     * @brief Set the current user
     * 
     * @param username name of the user
     */
    void set_user(std::string username);

    /**
     * @brief Compute the optimal path from a starting point to a destination point.
     * 
     * @param start
     * @param dest
     * @return ROUTE the route to the destination node or a route with length = -1 if
     * path generation was not successfull
     */
    ROUTE compute_path(sgd_util::LatLon start, sgd_util::LatLon dest);

    /**
     * @brief Compute the optimal path from a starting point to a destination point.
     * 
     * @param start 
     * @param dest_node_id 
     * @return ROUTE the route to the destination node or a route with length = -1 if
     * path generation was not successfull
     */
    ROUTE compute_path(sgd_util::LatLon start, llong dest_node_id);

    /**
     * @brief Find the nearest node to the specified lat/lon
     * 
     * @param latlon 
     * @return node id
     */
    llong node_near_lat_lon(sgd_util::LatLon latlon);

    std::vector<A_Star_Node> get_nodelist();

private:

    std::unordered_map<llong, A_Star_Node> nodelist_;       /// @brief map to store all nodes. Access via node id
    std::shared_ptr<A_Star_Users> users_ = nullptr;         /// @brief TODO

    std::unordered_map<llong, A_Star_Node> closedList;

    /**
     * @brief Traces the path and outputs all nodes in the correct order
     * 
     * @param closedList computed nodes from a star
     * @param dest destination node
     */
    ROUTE trace_path(std::unordered_map<llong, A_Star_Node> closedList, A_Star_Node dest);
};

}   // namespace nav_sgd

#endif