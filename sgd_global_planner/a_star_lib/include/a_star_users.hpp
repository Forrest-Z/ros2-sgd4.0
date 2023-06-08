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

#ifndef SGD_GLOBAL_PLANNER__A_STAR_USERS_HPP_
#define SGD_GLOBAL_PLANNER__A_STAR_USERS_HPP_

#include <string>
#include <unordered_map>
#include <vector>
#include <cmath>

#include "tinyxml2.h"
#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

namespace sgd_global_planner
{

class A_Star_Users
{

struct USER
{
    std::string name;
    std::unordered_map<std::string, double> factors;

    void add_factor(std::string key, std::string value)
    {
        PLOGD.printf("Add factor: '%s': %s", key.c_str(), value.c_str());
        try
        {
            double d = std::stod(value);
            factors.insert({key, d});
        }
        catch(const std::invalid_argument& e)
        {
            PLOGE.printf("Could not add factor to user %s: %s", name.c_str(), e.what());
        }
    }

    double get_factor(std::string key, std::string value)
    {
        if (factors.count(key) > 0)
        {
            return factors.at(key);
        }
        else if (factors.count(key + ":" + value) > 0)
        {
            return factors.at(key + ":" + value);
        }
        else if (factors.count(key + ":default") > 0)
        {
            return factors.at(key + ":default");
        }
        else
        {
            return -1.0;
        }
    }
};

private:
    std::string users_file_;
    std::unordered_map<std::string, USER> users;
    USER default_user;
    USER current_user;

    /**
     * @brief 
     * 
     * @param user 
     * @return USER 
     */
    USER read_user(tinyxml2::XMLElement * user);

public:
    /**
     * @brief Construct a new a star users object
     * 
     * @param users_file 
     */
    A_Star_Users(std::string users_file);
    ~A_Star_Users() {};

    /**
     * @brief Set current user to username. If username is not a valid user set default user.
     * 
     * @param username The username of the user
     */
    void set_user(std::string username);

    /**
     * @brief Calculate factor for given key
     * 
     * @param key 
     * @param value 
     * @return double the factor
     */
    double calculate_factor(std::string key, std::string value);

    /**
     * @brief 
     * 
     * @param angle 
     * @return double 
     */
    double angle_factor(double angle);

    /**
     * @brief Get the list of usernames
     * 
     * @return std::vector<std::string> 
     */
    std::vector<std::string> get_user_list();
};

} // namespace nav_sgd


#endif