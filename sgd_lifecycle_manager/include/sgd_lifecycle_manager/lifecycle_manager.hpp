#ifndef SGD_LIFECYCLE__LIFECYCLE_MANAGER_HPP_
#define SGD_LIFECYCLE__LIFECYCLE_MANAGER_HPP_

#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "tinyxml2.h"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace sgd_lifecycle
{

struct lifecycle_node
{
    std::string node_name;
    uint8_t state;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr srv_client;
    std::vector<std::string> depends;
};

struct group
{
    std::string group_name;
    std::vector<std::string> node_names;
};


class Lifecycle_Manager : public rclcpp::Node
{

public:
    Lifecycle_Manager();
    ~Lifecycle_Manager();

protected:
    // parameters
    std::string launch_file;

    std::vector<group> launch_groups;

    void read_xml_file(tinyxml2::XMLElement * node, group *g = nullptr);
    std::vector<lifecycle_node> lifecycle_nodes;

    //! \brief Init subscriber
    void init_sub();
    rclcpp::TimerBase::SharedPtr timer_;

    bool change_state(uint8_t transition_id, lifecycle_node &lfnode);
    bool all_nodes_active();
    bool is_node_active(std::string node_name);
};

}

#endif
