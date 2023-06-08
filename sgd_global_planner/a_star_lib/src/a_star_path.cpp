// #include "a_star_path.hpp"
#include "../include/a_star_path.hpp"

namespace sgd_global_planner
{

A_Star_Path::A_Star_Path(llong dest_node_id) : dest_node_id_(dest_node_id) {}

void
A_Star_Path::add_attribute(std::string key, std::string value)
{
    attributes_.insert({key, value});
}

double
A_Star_Path::cost(std::shared_ptr<A_Star_Users> users)
{
    double c = 1.0;
    for (auto att : attributes_)
    {
        c *= users->calculate_factor(att.first, att.second);
    }
    return c;
}

llong 
A_Star_Path::get_dest()
{
    return dest_node_id_;
}
    
} // namespace sgd_global_planner
