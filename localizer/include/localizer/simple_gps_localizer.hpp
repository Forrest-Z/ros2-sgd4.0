
#ifndef SIMPLE_GPS_LOCLIZER_HPP_
#define SIMPLE_GPS_LOCALIZER_HPP_

#include <utility>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "sgd_util/sgd_util.hpp"


namespace sgd
{

class Simple_Gps_Localizer : public nav2_util::LifecycleNode
{

public:
    Simple_Gps_Localizer();
    ~Simple_Gps_Localizer();

protected:
    // Implement the lifecycle interface
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
    
    //! \brief Init parameters
    void init_paramters();
    std::string target_frame_;
    std::string source_frame_;
    std::string gps_topic_;
    std::string odom_topic_;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_gps_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;

    //! \brief Init transforms
    void init_transforms();
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped tf_map_odom_;

    // static transform from base_footprint to gps_link
    double x_base_gps_, y_base_gps_;
    double x_odom_, y_odom_, angle_odom_;
    bool is_odom_avail_;

    nav2_util::CallbackReturn wait_for_transform();

    void gps_sub_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg_);
    void odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg_);

    inline geometry_msgs::msg::Quaternion angleZ_to_Quaternion(double angle)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, angle);
        return tf2::toMsg(q);
    }
};

}   // namespace sgd



#endif
