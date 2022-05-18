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

#include "sgd_util/geotools.hpp"

namespace nav_sgd
{

class A_Star_Node;
typedef int64_t llong;

class A_Star_Path
{
private:
    //llong dest_id_;
    A_Star_Node *dest_node_;
    std::unordered_map<std::string, std::string> attributes_;
public:
    A_Star_Path(A_Star_Node *dest_node)
    {
        dest_node_ = dest_node;
    }
    ~A_Star_Path() {};

    void add_attribute(std::string key, std::string value)
    {
        attributes_.insert({key, value});
    }

    double cost(std::shared_ptr<A_Star_Users> users)
    {
        double c = 1.0;

        for (auto att : attributes_)
        {
            c *= users->calculate_factor(att.first, att.second);
        }
        return c;
    }

    A_Star_Node * get_dest()
    {
        return dest_node_;
    }
};

class A_Star_Node
{
private:
    llong id_;
    bool is_blocked_;
    sgd_util::LatLon latlon_;
    
    llong predecessor_id_ = 0;
    double g_ = 0, h_ = 0;
    //double cost_;

    A_Star_Node *predecessor_ = nullptr;
    std::vector<A_Star_Path> paths_;

    /**
     * @brief Stores neighbor nodes with cost factors.
     */
    //std::unordered_map<llong, double> neighbours;

public:
    A_Star_Node() = default;
    A_Star_Node(llong id, sgd_util::LatLon coords)
            : id_(id), latlon_(coords), is_blocked_(false) { }
    ~A_Star_Node() = default;

    /**
     * @brief Set the value for the heuristic
     * 
     * @param h 
     */
    void set_h(double h) {h_ = h;}
    double f() {return g_ + h_;}
    llong id() {return id_;}

    std::vector<A_Star_Path> paths()
    {
        return paths_;
    }

    /**
     * @brief Set the predecessor node. Calculates g and h.
     * 
     * @param predecessor the predecessing node
     * @return g (cost to predecessor)
     */
    double set_predecessor(A_Star_Node *predecessor, double cost, double angle_cost)
    {
        // get predecessor from neighbours
        predecessor_ = predecessor;
        g_ = predecessor->g_ + latlon_.distance_to(predecessor->latlon_) * cost * angle_cost;

        return g_;
    }

    /**
     * @brief Get the path angle object
     * 
     * @return double 
     */
    double get_path_angle()
    {
        if (predecessor_ != nullptr)
        {
            return predecessor_->latlon_.bearing(latlon_);
        }
        return 0.0;
    }

    /**
     * @brief Check if this node has the "blocked" attribute set
     * 
     * @return true if this node is blocked, otherwise false
     */
    bool is_blocked() {return is_blocked_;};

    /**
     * @brief 
     * 
     * @return long 
     */
    llong predecessor_id()
    {
        if (predecessor_ != nullptr)
        {
            return predecessor_->id();
        }
        return 0;
    }        
    
    /**
     * @brief Get the coordinates for this node.
     * 
     * @return sgd_util::LatLon 
     */
    sgd_util::LatLon get_latlon() {return latlon_;}

    /**
     * @brief Add a neighbour node to this node.
     * 
     * @param id id of the neighbour node
     * @param cost the cost factor for the way to the neighbour node
     */
    //void add_neighbour(llong id, double cost) {neighbours.insert({id, cost});}

    void add_neighbour(A_Star_Path path) {paths_.push_back(path);};

    /**
     * @brief Check if a node with the specified id is a neighbour of this node.
     * 
     * @param id the id of the potential neighbour
     * @return true if node has a neighbor with the specified id
     */
    //bool has_neighbour(llong id) {return neighbours.count(id) > 0;}

    inline bool operator==(const A_Star_Node& nd) { return id_ == nd.id_; }
    inline bool operator!=(const A_Star_Node& nd) { return id_ != nd.id_; }
};

} // namespace nav_sgd

#endif