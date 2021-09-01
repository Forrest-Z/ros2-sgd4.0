#ifndef NAVIGATOR_HPP_
#define NAVIGATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sgd_msgs/srv/compute_path.hpp"
#include "sgd_util/sgd_util.hpp"

namespace nav_sgd
{
class Navigator : public rclcpp::Node
{
private:
    double gps_lat_, gps_lon_, clicked_point_lat_, clicked_point_lon_;

    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_clicked_point_;
    rclcpp::Client<sgd_msgs::srv::ComputePath>::SharedPtr compute_path_client_;

    void sub_gps_handler(const sensor_msgs::msg::NavSatFix::SharedPtr msg_);
    void sub_clicked_point_handler(const geometry_msgs::msg::PointStamped::SharedPtr msg_);

public:
    Navigator();
    ~Navigator();
};

}       // namespace nav_sgd


#endif