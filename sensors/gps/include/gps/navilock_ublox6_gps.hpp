#ifndef NAVILOCK_UBLOX6_GPS_HPP_
#define NAVILOCK_UBLOX6_GPS_HPP_

#include <list>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav2_util/lifecycle_node.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sgd_msgs/msg/serial.hpp"
#include "gps/nmea_parser.hpp"
#include "sgd_util/sgd_util.hpp"

namespace sgd_sensors
{

class Navilock_UBlox6_GPS : public nav2_util::LifecycleNode
{

struct xy
{
    double x, y;
};

public: 
    Navilock_UBlox6_GPS();
    ~Navilock_UBlox6_GPS();

protected:
    // Implement the lifecycle interface
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    std::fstream out_gps_;
    double time_at_start_;
    std::string time_to_string();

    //! \brief Init parameters
    void init_parameters();
    std::string port_;
    std::string xml_file_;
    std::string output_folder_;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
    rclcpp::Subscription<sgd_msgs::msg::Serial>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    int gps_counter_;
    
    void init_transforms();
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::shared_ptr<Nmea_Parser> nmea_parser_;

    void read_msg(const sgd_msgs::msg::Serial::SharedPtr msg);
    double get_direction_from_previous();

    xy last_pos_;
};

}

#endif
