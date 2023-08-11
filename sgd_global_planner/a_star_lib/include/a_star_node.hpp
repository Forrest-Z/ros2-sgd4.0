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

#ifndef SGD_GLOBAL_PLANNER__A_STAR_NODE_HPP_
#define SGD_GLOBAL_PLANNER__A_STAR_NODE_HPP_

#include <unordered_map>
#include <memory>

#include "sgd_util/geotools.hpp"
#include "a_star_path.hpp"

namespace sgd_global_planner
{

typedef int64_t llong;  // format used to store the ids

class A_Star_Node
{
private:
    llong id_, pid_;
    sgd_util::LatLon latlon_;
    
    bool is_blocked_;
    double g_ = 0, h_ = 0;

    // A_Star_Node *predecessor_ = nullptr;

    // predessor data
    llong predecessor_id_ = 0;
    sgd_util::LatLon predecessor_latlon_;

    std::vector<A_Star_Path> paths_;

public:
    // A_Star_Node() = default;
    A_Star_Node(llong id, sgd_util::LatLon coords)
            : id_(id), latlon_(coords), is_blocked_(false) { }
    // A_Star_Node(const A_Star_Node &node);
    ~A_Star_Node() = default;

    /**
     * @brief Set the value for the heuristic
     * 
     * @param h 
     */
    void set_h(double h);

    /**
     * @brief 
     * 
     * @return double 
     */
    double f();

    /**
     * @brief 
     * 
     * @return llong 
     */
    llong id();

    /**
     * @brief Set the parent id
     * 
     * @param pid 
     */
    void set_pid(llong pid);

    /**
     * @brief Return the parent id if it is set, otherwise the id
     * 
     * @return llong 
     */
    llong pid();

    std::vector<A_Star_Path> paths();

    /**
     * @brief Set the predecessor node. Calculates g (cost to predecessor)
     * 
     * @param predecessor the predecessing node
     * @param cost cost factor for way
     * @param angle_cost angle cost factor
     * @return g (cost to predecessor)
     */
    double set_predecessor(A_Star_Node predecessor, double cost, double angle_cost);

    /**
     * @brief Get the bearing from predecessor node to this node
     * 
     * @return double 
     */
    double get_predecessor_bearing();

    /**
     * @brief Check if this node has the "blocked" attribute set
     * 
     * @return true if this node is blocked, otherwise false
     */
    bool is_blocked();

    /**
     * @brief 
     * 
     * @return long 
     */
    llong predecessor_id();

    /**
     * @brief Get the coordinates for this node.
     * 
     * @return sgd_util::LatLon 
     */
    sgd_util::LatLon get_latlon();

    /**
     * @brief 
     * 
     * @param path 
     */
    void add_neighbour(A_Star_Path path);

    /**
     * @brief Reset node
     * 
     */
    void reset();

    inline bool operator==(const A_Star_Node& nd) { return id_ == nd.id_; }
    inline bool operator!=(const A_Star_Node& nd) { return id_ != nd.id_; }
    // A_Star_Node& operator=(const A_Star_Node& nd);
};

} // namespace sgd_global_planner

#endif