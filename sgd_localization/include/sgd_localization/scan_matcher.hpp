// Copyright 2023 HAW Hamburg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SGD_LOCALIZATION_SCAN_MATCHER_HPP
#define SGD_LOCALIZATION_SCAN_MATCHER_HPP

#include <chrono>
#include <queue>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sgd_msgs/msg/vec_obstacle_array.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/utils.h"

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

// #include "obstacle.hpp"
#include "line.hpp"
// #include "point.hpp"

namespace sgd_localization
{

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ScanMatcher : public rclcpp_lifecycle::LifecycleNode
{
public:
    ScanMatcher();
    ~ScanMatcher();

protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    // /**
    //  * @brief Initialize parameters for this node
    //  * 
    //  */
    // void init_parameters();
    // std::string port_;

    uint32_t obstacle_id;
    bool print_debug_;
    float max_angle_diff = M_PI / 8.0;

    // publisher and subscriber
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Subscription<sgd_msgs::msg::VecObstacleArray>::SharedPtr sub_local_map_;
    rclcpp_lifecycle::LifecyclePublisher<sgd_msgs::msg::VecObstacleArray>::SharedPtr pub_obstacles;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Quaternion>::SharedPtr pub_orientation;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    geometry_msgs::msg::TransformStamped tf_map_scan;
    
    /**
     * @brief Handle received scan message
     * 
     * @param msg received laser scan message
     */
    void on_scan_received(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    rclcpp::Time last_msg;

    /**
     * 
    */
    void on_local_map_received(const sgd_msgs::msg::VecObstacleArray::SharedPtr msg);
    sgd_msgs::msg::VecObstacleArray local_obstacles_;
    

    /**
     * @brief Compute linear regression for the specified subset of msg
     * 
     * @param msg subset with ranges in x-y-coordinates
     * @param start start index of subset (included)
     * @param end end index of subset (not included)
     * @return std::vector<float> 
     */
    std::vector<std::pair<Point, int>> split_and_merge(std::vector<Point> subset, int start, int end, int depth);
    
    Point calc_intersection(Point p1, Point p2, Point p3, Point p4);

    /**
     * @brief 
     * 
     * @param pl1 1. Stuetzpunkt der Geraden
     * @param pl2 second Stuetzpunkt der Geraden
     * @param p point to calculate distance to
     * @return float 
     */
    float calc_distance_to_line(Point pl1, Point pl2, Point p);

    /**
     * @brief Returns the nearest three lines 
    */
    Line get_nearest_from_local(Line l);

    inline bool is_valid(float measurement)
    {
        // TODO: parameterized function
        return !(measurement < 0.0) && !(25.0 < measurement);
    }
};
}

#endif