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

#ifndef NAV_SGD__A_STAR_NODE_HPP_
#define NAV_SGD__A_STAR_NODE_HPP_

#include <unordered_map>
#include <memory>

#include "a_star_users.hpp"
#include "tinyxml2.h"
#include "sgd_util/geotools.hpp"

namespace nav_sgd
{

class A_Star_Node
{
private:
    std::shared_ptr<A_Star_Users> users_;
    sgd_util::LatLon latlon_;

    long id_, predecessor_id_;    
    double g_ = 0, h_ = 0;
    double cost_;

    /**
     * @brief Stores neighbor nodes with cost factors.
     */
    std::unordered_map<long, double> neighbors;

    double calc_cost_factor(tinyxml2::XMLElement *node);

public:
    A_Star_Node() = default;
    A_Star_Node(tinyxml2::XMLElement *node, std::shared_ptr<A_Star_Users> users);
    A_Star_Node(tinyxml2::XMLElement *node, A_Star_Node predecessor, std::shared_ptr<A_Star_Users> users);
    ~A_Star_Node();

    void set_h(double h);
    double f();
    long id();
    long predecessor_id();
    sgd_util::LatLon get_latlon();

    /**
     * @brief 
     * 
     * @param id 
     * @return true if node has a neighbor with the specified id
     */
    bool has_neighbor(long id);

    inline bool operator==(const A_Star_Node& nd) { return id_ == nd.id_; }
    inline bool operator!=(const A_Star_Node& nd) { return id_ != nd.id_; }
};

} // namespace nav_sgd

#endif