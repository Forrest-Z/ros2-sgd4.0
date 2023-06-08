#include "../include/a_star_users.hpp"

namespace sgd_global_planner
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
        PLOGE.printf("Error reading xml file %s: %s", users_file.c_str(), doc.ErrorStr());
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
        PLOGE << "No default user found!";   // TODO error handling
    }
}

void
A_Star_Users::set_user(std::string username)
{
    current_user = users.count(username) > 0 ? users[username] : default_user;
}

double
A_Star_Users::calculate_factor(std::string key, std::string value)
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

double
A_Star_Users::angle_factor(double angle)
{
    // double a = 0.5;
    // double b = 1.0;
    // double c = 0.1;

    double a = current_user.get_factor("angle", "a");
    double b = current_user.get_factor("angle", "b");
    double c = current_user.get_factor("angle", "c");

    return (a * exp(-pow(b * angle, 2)) * angle + c * tanh(5.0 * angle) + 1);
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

A_Star_Users::USER
A_Star_Users::read_user(tinyxml2::XMLElement * user)
{
    USER u;
    if (user->Attribute("name") != NULL)
    {
        u.name = user->Attribute("name");
    } else {
        u.name = "default";
    }
    PLOGD.printf("Read user with name %s", u.name.c_str());

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

} // namespace sgd_global_planner