//#include "a_star_node.hpp"
#include "../include/a_star_node.hpp"

namespace sgd_global_planner
{

// A_Star_Node::A_Star_Node(const A_Star_Node &node)
//         : id_(node.id_), latlon_(node.latlon_), is_blocked_(node.is_blocked_)
// {
//     paths_ = node.paths_;

//     predecessor_id_ = node.predecessor_id_;
//     predecessor_latlon_ = node.predecessor_latlon_;
// }

void
A_Star_Node::set_h(double h) {h_ = h;}

double
A_Star_Node::f()
{
    return g_ + h_;
}

llong
A_Star_Node::id()
{
    return id_;
}

void
A_Star_Node::set_pid(llong pid)
{
    pid_ = pid;
}

llong
A_Star_Node::pid()
{
    return pid_ > 0 ? pid_ : id_;
}

std::vector<A_Star_Path>
A_Star_Node::paths()
{
    return paths_;
}

double
A_Star_Node::set_predecessor(A_Star_Node predecessor, double cost, double angle_cost)
{
    // get predecessor from neighbours
    // predecessor_ = predecessor;
    predecessor_id_ = predecessor.id_;
    predecessor_latlon_ = predecessor.get_latlon();
    g_ = predecessor.g_ + latlon_.distance_to(predecessor_latlon_) * cost * angle_cost;

    return g_;
}

double
A_Star_Node::get_predecessor_bearing()
{
    return (predecessor_id_ > 0) ? predecessor_latlon_.bearing(latlon_) : 0.0;
}

bool
A_Star_Node::is_blocked()
{
    return is_blocked_;
}

llong
A_Star_Node::predecessor_id()
{
    return predecessor_id_;
}

sgd_util::LatLon
A_Star_Node::get_latlon()
{
    return latlon_;
}

void
A_Star_Node::add_neighbour(A_Star_Path path)
{
    paths_.push_back(path);
}

void
A_Star_Node::reset()
{
    predecessor_id_ = 0;

    g_ = 0.0;
    h_ = 0.0;
}

// A_Star_Node&
// A_Star_Node::operator=(const A_Star_Node &nd)
// {
//     if (this == &nd)
//         return *this;

//     A_Star_Node node(nd);
//     return node;
// }

} // namespace sgd_global_planner
