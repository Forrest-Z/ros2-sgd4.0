// Copyright 2022 HAW Hamburg
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

#ifndef SGD_LOCAL_COSTMAP_OBST_DETECTOR_HPP
#define SGD_LOCAL_COSTMAP_OBST_DETECTOR_HPP

#include <chrono>

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

#include "obstacle.hpp"

namespace sgd_local_costmap
{

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ObstacleDetector : public rclcpp_lifecycle::LifecycleNode
{
public:
    ObstacleDetector();
    ~ObstacleDetector();

protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    void init_parameters();
    std::string port_;
    std::string scan_topic;

    sgd_msgs::msg::VecObstacleArray obstacle_message;
    std::vector<Obstacle> obstacles;
    uint32_t obstacle_id;
    bool print_debug_;

    // publisher and subscriber
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp_lifecycle::LifecyclePublisher<sgd_msgs::msg::VecObstacleArray>::SharedPtr pub_obstacles;

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
     * @brief Compute linear regression for the specified subset of msg
     * 
     * @param msg subset with ranges in x-y-coordinates
     * @param start start index of subset (included)
     * @param end end index of subset (not included)
     * @return std::vector<float> 
     */
    std::vector<Point> split_and_merge(std::vector<Point> subset, int start, int end, int depth);
    
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

    inline bool is_valid(float measurement)
    {
        // TODO: parameterized function
        return !(measurement < 0.0) && !(25.0 < measurement);
    }
};
}

#endif