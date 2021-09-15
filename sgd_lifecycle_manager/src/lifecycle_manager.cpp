#include "sgd_lifecycle_manager/lifecycle_manager.hpp"

namespace sgd_lifecycle
{

using namespace std::chrono_literals;   // if a timer is used

Lifecycle_Manager::Lifecycle_Manager():
    Node("lifecycle_manager")
{
    RCLCPP_INFO(get_logger(), "Creating");

    launch_file = declare_parameter<std::string>("launch_file", "/params/launch/launch.xml");

    std::ifstream t(launch_file);
    std::cout << "File to read: " << launch_file << std::endl;

    // read launch xml file
    std::vector<char> buffer = std::vector<char>((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    buffer.push_back('\0');
    rapidxml::xml_document<> doc;
    doc.parse<0>(&buffer[0]);

    root = doc.first_node(0);   // root element of the xml file
    read_xml_file(root);


    /*
        - read and parse launch file
        - create listener for each node/transition_event topic
        - get state from nodes
    */

   // TODO create timer??


}

Lifecycle_Manager::~Lifecycle_Manager()
{
    // Destroy
}

void
Lifecycle_Manager::read_xml_file(rapidxml::xml_node<> *node)
{
    std::cout << node->name() << std::endl;
    for (rapidxml::xml_node<> *nd = node->first_node(); nd; nd = nd->next_sibling())
    {
        std::cout << nd->name() << std::endl;
        if (nd->name() == "node")
        {
            std::string name(nd->first_attribute("name")->value());
            states.insert(std::make_pair(name, State::PRIMARY_STATE_UNCONFIGURED));
        }
        else
        {
            read_xml_file(nd);
        }
    }
}

void
Lifecycle_Manager::init_sub()
{
    //RCLCPP_DEBUG(get_logger(), "Init publisher and subscriber");
    //example_sub_ = create_subscription<package::msg::MessageType>(
    //    "topic_name", default_qos, std::bind(&Example_Node::on_msg_received, this, std::placeholders::_1));
}

}   // namespace sgd_lifecycle

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_lifecycle::Lifecycle_Manager>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
