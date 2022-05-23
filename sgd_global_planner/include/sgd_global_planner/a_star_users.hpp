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

#ifndef NAV_SGD_A_STAR_USERS_HPP_
#define NAV_SGD_A_STAR_USERS_HPP_

#include <string>
#include <unordered_map>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>

#include "tinyxml2.h"

namespace nav_sgd
{

class A_Star_Users
{

struct USER
{
    std::string name;
    std::unordered_map<std::string, double> factors;

    void add_factor(std::string key, std::string value)
    {
        try
        {
            double d = std::stod(value);
            factors.insert({key, d});
        }
        catch(const std::invalid_argument& e)
        {
            std::cout << "Could not add factor to user " << name << std::endl;
            std::cerr << e.what() << '\n';
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
            return 1E6;
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
    USER read_user(tinyxml2::XMLElement * user)
    {
        USER u;
        if (user->Attribute("name") != NULL)
        {
            u.name = user->Attribute("name");
        } else {
            u.name = "default";
        }

        auto nd = user->FirstChildElement();
        while (nd != NULL)
        {
            if (nd->FirstChildElement())
            {
                std::string prefix = nd->Value();
                // gehe durch alle child nodes
                auto nn = nd->FirstChildElement();
                while (nn != NULL)
                {
                    u.add_factor(prefix + ":" + nn->Value(), nn->GetText());
                    nn = nn->NextSiblingElement();
                }
            }
            else
            {
                u.add_factor(nd->Value(), nd->GetText());
            }
            nd = nd->NextSiblingElement();
        }
        return u;
    }

public:
    /**
     * @brief Construct a new a star users object
     * 
     * @param users_file 
     */
    A_Star_Users(std::string users_file)
    {
        // read xml
        tinyxml2::XMLDocument doc;
        doc.LoadFile(users_file.c_str());
        auto root = doc.RootElement();      // <users>

        // check error
        if (doc.ErrorID())
        {
            // TODO Error handling
            std::cerr << doc.ErrorStr();
            return;
        }

        // go through xml and add users
        bool has_default_user = false;
        auto usr = root->FirstChildElement("user");
        while (usr != NULL)
        {
            // new user
            USER user = read_user(usr);
            if (user.name.compare("default") == 0)
            {
                has_default_user = true;
                default_user = user;
            }
            else
            {
                users.insert({user.name, user});
            }
            usr = usr->NextSiblingElement();
        }
        if (!has_default_user)
        {
            std::cerr << "No default user found!\n";   // TODO error handling
        }
    }

    ~A_Star_Users() {};

    /**
     * @brief Set current user to username. If username is not a valid user set default user.
     * 
     * @param username The username of the user
     */
    void set_user(std::string username)
    {
        current_user = users.count(username) > 0 ? users[username] : default_user;
    }

    /**
     * @brief Calculate factor for given key
     * 
     * @param key 
     * @param value 
     * @return double the factor
     */
    double calculate_factor(std::string key, std::string value)
    {
        // try to get factor for current user
        double f = -1.0;
        if (key == "angle")
        {
            // do nothing
            f = 1.0;
        }
        else
        {
            f = current_user.get_factor(key, value);    // bsp. highway, footway
        }

        if (f > 0)
            return f;
        
        // get value from default user
        f = default_user.get_factor(key, value);

        return f < 0 ? 1E6 : f;
    }

    /**
     * @brief 
     * 
     * @param angle 
     * @return double 
     */
    double angle_factor(double angle)
    {
        double a = 0.5;
        double b = 1.0;
        double c = 0.1;

        return (a * exp(-pow(b * angle, 2)) * angle + c * tanh(5.0 * angle) + 1);
    }

    /**
     * @brief Get the list of usernames
     * 
     * @return std::vector<std::string> 
     */
    std::vector<std::string> get_user_list()
    {
        std::vector<std::string> userlist;
        for (auto usr : users) {
            userlist.push_back(usr.first);
        }
        return userlist;
    }
};

} // namespace nav_sgd


#endif