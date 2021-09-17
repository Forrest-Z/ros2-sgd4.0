
#ifndef SGD_LOCALIZER__INITIAL_POSE_ESTIMATOR_HPP_
#define SGD_LOCALIZER__INITIAL_POSE_ESTIMATOR_HPP_

#include <utility>
#include <math.h> 
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "sgd_util/sgd_util.hpp"


namespace sgd_localizer
{

class Initial_Pose_Estimator : public nav2_util::LifecycleNode
{

public:
    Initial_Pose_Estimator();
    ~Initial_Pose_Estimator();

protected:
    // Implement the lifecycle interface
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
    
    //! \brief Init parameters
    void init_parameters();
    std::string source_frame_;
    std::string gps_topic_;
    std::string imu_topic_;
    std::string initial_pose_topic_;

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_gps_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initial_pose_;
    rclcpp::TimerBase::SharedPtr timer_;

    void on_gps_received(const sensor_msgs::msg::NavSatFix::SharedPtr msg_);
    void on_imu_received(const sensor_msgs::msg::Imu::SharedPtr msg_);

    void publish_initial_pose();
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    
    // indicator whether gps/imu signal is good enough to estimate pose
    bool gps_good, imu_good;
    bool initial_pose_set;
};

}   // namespace sgd_localizer



#endif
