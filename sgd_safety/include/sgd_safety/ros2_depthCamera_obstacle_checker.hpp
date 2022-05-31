#ifndef SGD_SAFETY__ROS_DEPTH_CAMERA_OOBSTACLE_CHECKER_HPP_
#define SGD_SAFETY__ROS_DEPTH_CAMERA_OOBSTACLE_CHECKER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sgd_safety/depthCamera_obstacle_checker.hpp"

namespace sgd_safety
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Ros2_DepthCamera_Obstacle_Checker : public rclcpp_lifecycle::LifecycleNode
{

protected:
    // Implement the lifecycle interface
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    //! \brief Init parameters
    
    std::string cmd_vel_depthCamera_topic_;
    

    double robot_width_;
    double distance_min_;
    double distance_max_;

    DepthCameraObstacleChecker dcam_oc;
    geometry_msgs::msg::Twist depthCamera_cmd_vel;
    

    //! \brief Init Publisher and subscriber
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr pub_depthCamera_cmd_vel;

public:
    Ros2_DepthCamera_Obstacle_Checker(void);
    ~Ros2_DepthCamera_Obstacle_Checker(void);
    Ros2_DepthCamera_Publish_Speed(void);
};

} 

#endif
