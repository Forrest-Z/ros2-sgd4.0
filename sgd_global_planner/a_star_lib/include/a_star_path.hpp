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

#ifndef SGD_GLOBAL_PLANNER__A_STAR_PATH_HPP_
#define SGD_GLOBAL_PLANNER__A_STAR_PATH_HPP_

#include <string>
#include <unordered_map>
#include <memory>

#include "a_star_users.hpp"

namespace sgd_global_planner
{

typedef int64_t llong;  // format used to store the ids

class A_Star_Path
{
private:
    llong dest_node_id_;
    std::unordered_map<std::string, std::string> attributes_;
public:
    /**
     * @brief Construct a new path to node with id dest_node_id
     * 
     * @param dest_node_id 
     */
    A_Star_Path(llong dest_node_id);
    ~A_Star_Path() {};

    /**
     * @brief add attribute to path
     * 
     * @param key 
     * @param value 
     */
    void add_attribute(std::string key, std::string value);

    /**
     * @brief Calculate the cost factor along the path for the specified user
     * 
     * @param users the user
     * @return double cost along to path
     */
    double cost(std::shared_ptr<A_Star_Users> users);

    /**
     * @brief get id of destination node
     * 
     */
    llong get_dest();
};

} // namespace sgd_global_planner

#endif      // SGD_GLOBAL_PLANNER__A_STAR_PATH_HPP_