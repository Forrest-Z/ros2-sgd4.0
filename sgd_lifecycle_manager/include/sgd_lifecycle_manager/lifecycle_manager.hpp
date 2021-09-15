#ifndef SGD_LIFECYCLE__LIFECYCLE_MANAGER_HPP_
#define SGD_LIFECYCLE__LIFECYCLE_MANAGER_HPP_

#include <unordered_map>
#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rapidxml/rapidxml.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace sgd_lifecycle
{

class Lifecycle_Manager : public rclcpp::Node
{

public:
    Lifecycle_Manager();
    ~Lifecycle_Manager();

protected:
    // parameters
    std::string launch_file;

    void read_xml_file(rapidxml::xml_node<> *node);
    rapidxml::xml_node<> * root;
    std::unordered_map<std::string, uint8_t> states;

    //! \brief Init subscriber
    void init_sub();
};

}

#endif
