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

#ifndef SGD_HARDWARE__CAPACITIVE_TOUCH_HPP_
#define SGD_HARDWARE__CAPACITIVE_TOUCH_HPP_

#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sgd_msgs/msg/touch.hpp"
#include "sgd_io/serial.hpp"

#include "moving_average_filter.hpp"

namespace sgd_hardware_drivers 
{

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Capacitive_Touch : public rclcpp_lifecycle::LifecycleNode
{
public:
    Capacitive_Touch();
    ~Capacitive_Touch();

protected:
    // Implement the lifecycle interface
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Initialize parameters
     */
    void init_parameters();

    /**
     * @brief If the received value is greater than the threshold, it is assumed that the sensor is touched.
     */
    int thresh_;

    /**
     * @brief Size of the moving average filter
     */
    int filter_size_;

    
    /**
     * @brief Initialize publisher and subscriber.
     */
    void init_pub_sub();
    rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_lifecycle::LifecyclePublisher<sgd_msgs::msg::Touch>::SharedPtr publisher_;

    /**
     * @brief Class to read from / write to serial port.
     */
    sgd_io::Serial serial;

    /**
     * @brief Read data from serial port. Bind this method to timer.
     */
    void read_serial();

    /**
     * @brief Regular expression to use to get values from received message.
     */
    std::regex regex_;
    
    /**
     * @brief Moving average filter for right sensor.
     */
    MovingAverageFilter filter_r;

    /**
     * @brief Moving average filter for left sensor.
     */
    MovingAverageFilter filter_l;
};

}

#endif
