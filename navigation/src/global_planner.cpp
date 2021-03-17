

#include "navigation/global_planner.hpp"

#include <memory>

namespace nav_sgd
{

Global_Planner_OSM::Global_Planner_OSM():
        Node("global_planner_osm")
{
    // Do something
}

Global_Planner_OSM::~Global_Planner_OSM()
{
    // Destroy
}


void
Global_Planner_OSM::computePath(std::shared_ptr<sgd_msgs::srv::ComputePath::Request> request,
        std::shared_ptr<sgd_msgs::srv::ComputePath::Response> response)
{
    /*
    1. 
    */
}



}   // namespace nav_sgd

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("compute_path_server");

    rclcpp::Service<sgd_msgs::srv::ComputePath>::SharedPtr service = 
        node->create_service<sgd_msgs::srv::ComputePath>("compute_path", &nav_sgd::Global_Planner_OSM::computePath);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to compute path");

    rclcpp::spin(node);
    rclcpp::shutdown();
}




