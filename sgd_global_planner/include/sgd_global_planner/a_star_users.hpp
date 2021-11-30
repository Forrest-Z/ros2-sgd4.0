#ifndef NAV_SGD_A_STAR_USERS_HPP_
#define NAV_SGD_A_STAR_USERS_HPP_

#include <string>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

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
            factors.insert(std::pair(key, d));
        }
        catch(const std::invalid_argument& e)
        {
            std::cout << "Could not add factor to user " << name << std::endl;
            std::cerr << e.what() << '\n';
        }
    }
    double get_factor(std::string key, std::string value)
    {
        if (factors.count(key) > 0) {
            return factors[key];
        } else if (factors.count(key + ":" + value) > 0) {
            return factors[key + ":" + value];
        } else {
            return -1.0;
        }
    }
};

private:
    const std::string users_file_;
    std::unordered_map<std::string, USER> users;
    USER default_user;
    USER current_user;

    USER read_user(tinyxml2::XMLElement * user);
public:
    A_Star_Users(std::string users_file);
    ~A_Star_Users();

    //! \brief Set current user to username. If username is not a valid user set default user.
    //! \param username The username of the user
    void set_user(std::string username);
    //! \brief Calculate factor for given key
    //! \param key
    //! \param value
    //! \returns 
    double calculate_factor(std::string key, std::string value);
};

A_Star_Users::A_Star_Users(std::string users_file)
    : users_file_(users_file)
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
    while (usr)
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
            users.insert(std::pair(user.name, user));
        }
        usr = usr->NextSiblingElement();
    }
    if (!has_default_user)
    {
        std::cerr << "No default user found!\n";   // TODO error handling
    }    
}

A_Star_Users::~A_Star_Users()
{
    // destructor
}

A_Star_Users::USER
A_Star_Users::read_user(tinyxml2::XMLElement * user)
{
    USER u;
    u.name = user->Attribute("name");

    auto nd = user->FirstChildElement();
    while (nd)
    {
        if (nd->FirstChildElement())
        {
            std::string prefix = nd->Value();
            // gehe durch alle child nodes
            auto nn = nd->FirstChildElement();
            while (nn)
            {
                u.add_factor(prefix + ":" + nd->Value(), nd->ToText()->Value());
                nn = nn->NextSiblingElement();
            }
        }
        else
        {
            u.add_factor(nd->Value(), nd->ToText()->Value());
        }
        nd = nd->NextSiblingElement();
    }
    return u;   
}

double
A_Star_Users::calculate_factor(std::string key, std::string value)
{
    // try to get factor 
    double f = current_user.get_factor(key, value);

    if (f > 0) return f;
    
    f = default_user.get_factor(key, value);
    return f < 0 ? 1E6 : f;
}

void
A_Star_Users::set_user(std::string username)
{
    current_user = users.count(username) > 0 ? users[username] : default_user;
}

} // namespace nav_sgd


#endif