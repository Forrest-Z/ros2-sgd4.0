
#ifndef GLOBAL_PLANNER_HPP_
#define GLOBAL_PLANNER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sgd_msgs/srv/compute_path.hpp"

namespace nav_sgd
{

class Global_Planner_OSM : public rclcpp::Node
{
public:
    Global_Planner_OSM();
    ~Global_Planner_OSM();

    void computePath(const std::shared_ptr<sgd_msgs::srv::ComputePath::Request> request,
            std::shared_ptr<sgd_msgs::srv::ComputePath::Response> response);

protected:
    

};



}   // namespace nav_sgd



#endif
