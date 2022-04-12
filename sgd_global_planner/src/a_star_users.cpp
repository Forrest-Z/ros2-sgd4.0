#include "sgd_global_planner/a_star_users.hpp"

namespace nav_sgd
{

A_Star_Users::A_Star_Users(std::string users_file)
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

A_Star_Users::~A_Star_Users()
{
    // destructor
}

A_Star_Users::USER
A_Star_Users::read_user(tinyxml2::XMLElement * user)
{
    USER u;
    if (user->Attribute("name") != NULL)
    {
        u.name = user->Attribute("name");
    } else {
        u.name = "noname";
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

double
A_Star_Users::calculate_factor(std::string key, std::string value)
{
    // try to get factor 
    double f = current_user.get_factor(key, value);

    if (f > 0) return f;
    
    f = default_user.get_factor(key, value);
    return f < 0 ? 1E6 : f;
}

std::vector<std::string>
A_Star_Users::get_user_list()
{
    std::vector<std::string> userlist;
    for (auto usr : users) {
        userlist.push_back(usr.first);
    }
    return userlist;
}

void
A_Star_Users::set_user(std::string username)
{
    current_user = users.count(username) > 0 ? users[username] : default_user;
}

}