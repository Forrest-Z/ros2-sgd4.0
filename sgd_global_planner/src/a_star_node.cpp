#include "sgd_global_planner/a_star_node.hpp"

namespace nav_sgd
{

A_Star_Node::A_Star_Node(tinyxml2::XMLElement *node, std::shared_ptr<A_Star_Users> users)
{
    // set users
    users_ = users;

    // get id
    id_ = strtol(node->Attribute("id"), NULL, 10);
    predecessor_id_ = -1;
    cost_ = calc_cost_factor(node);
    
    latlon_.set_global_coordinates(strtod(node->Attribute("lat"), NULL), strtod(node->Attribute("lon"), NULL));

    // get neighbour nodes
    tinyxml2::XMLElement *nd = node->FirstChildElement("nd"); // get first neighbor node
    while (nd != NULL)
    {
        neighbors.insert({strtol(nd->Attribute("ref"), NULL, 10),
                            calc_cost_factor(nd)});

        nd = nd->NextSiblingElement();
    }
}

A_Star_Node::A_Star_Node(tinyxml2::XMLElement *node, A_Star_Node predecessor, std::shared_ptr<A_Star_Users> users)
    : A_Star_Node(node, users)
{
    predecessor_id_ = predecessor.id();

    g_ = predecessor.g_ + latlon_.distance_to(predecessor.latlon_) * cost_;
}

A_Star_Node::~A_Star_Node()
{
}

double
A_Star_Node::f()
{
    return g_ + h_;
}

void
A_Star_Node::set_h(double h)
{
    h_ = h;
}

long
A_Star_Node::id()
{
    return id_;
}

long
A_Star_Node::predecessor_id()
{
    return predecessor_id_;
}

sgd_util::LatLon
A_Star_Node::get_latlon()
{
    return latlon_;
}

bool
A_Star_Node::has_neighbor(long id)
{
    return neighbors.count(id) > 0;
}

double
A_Star_Node::calc_cost_factor(tinyxml2::XMLElement *node)
{
    // gehe durch alle child nodes
    // -> get name and value
    double f = 0;
    tinyxml2::XMLElement *n = node->FirstChildElement("node");
    while (n)
    {
        f += users_->calculate_factor(n->Value(), n->ToText()->Value());
        n = n->NextSiblingElement();
    }
    return f <= 0 ? 1E6 : f;
}

}