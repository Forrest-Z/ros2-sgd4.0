#include "sgd_lifecycle_manager/lifecycle_manager.hpp"

namespace sgd_lifecycle
{

using namespace std::chrono_literals;   // if a timer is used

Lifecycle_Manager::Lifecycle_Manager():
    Node("lifecycle_manager")
{
    launch_file = declare_parameter<std::string>("launch_file", "/params/launch.xml");

    tinyxml2::XMLDocument doc;
    doc.LoadFile(launch_file.c_str());

    if (doc.ErrorID() != 0)
    {
        RCLCPP_ERROR(get_logger(), doc.ErrorStr());
        return;
    }

    auto root = doc.RootElement();     // <launch>
    read_xml_file(root);
        
    // change node state to configure
    for (auto it = lifecycle_nodes.begin(); it != lifecycle_nodes.end(); it++)
    {
        if(change_state(Transition::TRANSITION_CONFIGURE, *it))
        {
            it->state = Transition::TRANSITION_CONFIGURE;
        }
    }

    int retries = 0;
    rclcpp::WallRate loop_rate(10);
    while (!all_nodes_active() && retries < 10)
    {
        retries++;
        for (auto it = lifecycle_nodes.begin(); it != lifecycle_nodes.end(); it++)
        {
            if (it->state == Transition::TRANSITION_ACTIVATE)   continue;

            bool depend_active = true;
            for (auto n : it->depends)
            {
                depend_active = depend_active && is_node_active(n);
            }

            if (depend_active)
            {
                if(change_state(Transition::TRANSITION_ACTIVATE, *it))
                {
                    it->state = Transition::TRANSITION_ACTIVATE;
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Could not activate node %s", it->node_name.c_str());
                    return;
                }
            }
            else
            {
                RCLCPP_DEBUG(get_logger(), "Node %s waiting for dependencies to become active.", it->node_name.c_str());
            }
        }
        if (!loop_rate.sleep())
        {
            RCLCPP_INFO(get_logger(), "Lifecycle Manager missed frequency.");
        }
        // TODO timeout
    }

    RCLCPP_INFO(get_logger(), "All nodes active.");
}

Lifecycle_Manager::~Lifecycle_Manager()
{
    // Destroy
}

void
Lifecycle_Manager::read_xml_file(tinyxml2::XMLElement * node, group *g)
{
    tinyxml2::XMLElement * nd = node->FirstChildElement();
    while (nd)
    {
        if (strcmp(nd->Value(), "node"))
        {
            lifecycle_node n;
            n.node_name = nd->Attribute("name");
            n.state = State::PRIMARY_STATE_UNCONFIGURED;
            n.srv_client = this->create_client<lifecycle_msgs::srv::ChangeState>(n.node_name + "/change_state");

            // add node name to group if 
            if (g != nullptr)   g->node_names.push_back(n.node_name);

            auto depend = nd->FirstChildElement("depend");
            while (depend)
            {
                // TODO add depending nodes
                depend = depend->NextSiblingElement("depend");
            }
            lifecycle_nodes.push_back(n);
        }
        else if (strcmp(nd->Value(), "group"))
        {
            group g;
            g.group_name = nd->Attribute("name");

            read_xml_file(nd, &g);

            launch_groups.push_back(g);
        }
        else
        {
            read_xml_file(nd);
        }
        nd = nd->NextSiblingElement();
    }
}

bool
Lifecycle_Manager::change_state(uint8_t transition_id, lifecycle_node &lfnode)
{
    RCLCPP_INFO(get_logger(), "Change state for node %s", lfnode.node_name.c_str());
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition_id;

    try
    {
        uint8_t retries = 0;
        while (!lfnode.srv_client->wait_for_service(1s)) {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            else if (retries > 5)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Maximum number of retries reached. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            retries++;
        }
    }
    catch(const std::runtime_error& e)
    {
        std::cerr << e.what() << '\n';
    }

    auto result = lfnode.srv_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        return result.get()->success;
    }

    return false;
}

bool
Lifecycle_Manager::all_nodes_active()
{
    for (auto n : lifecycle_nodes)
    {
        if( n.state != Transition::TRANSITION_ACTIVATE )
        {
            return false;
        }
    }
    return true;
}

bool
Lifecycle_Manager::is_node_active(std::string node_name)
{
    for (auto n : lifecycle_nodes)
    {
        if(n.node_name == node_name && n.state == Transition::TRANSITION_ACTIVATE)
        {
            return true;
        }
    }
    return false;
}

}   // namespace sgd_lifecycle

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_lifecycle::Lifecycle_Manager>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
