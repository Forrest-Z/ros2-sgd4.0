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

#include "cap_touch/capacitive_touch.hpp"

namespace sgd_hardware_drivers
{

Capacitive_Touch::Capacitive_Touch():
    rclcpp_lifecycle::LifecycleNode("capacitive_touch")
{
    // declare parameters with default values
    declare_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    declare_parameter("thresh", rclcpp::ParameterValue(2000));
    declare_parameter("filter_size", rclcpp::ParameterValue(4));
    declare_parameter("msg_regex", rclcpp::ParameterValue("Touch:(\\d*),R:(\\d*),L:(\\d*)"));
}

Capacitive_Touch::~Capacitive_Touch()
{
    // Destroy
}

CallbackReturn
Capacitive_Touch::on_configure(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    std::string port;
    get_parameter("port", port);
    serial.open_port(port, 9600);
    init_pub_sub();

    filter_r.set_filter_size(filter_size_);
    filter_l.set_filter_size(filter_size_);

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Capacitive_Touch::on_activate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Activating");
    publisher_->on_activate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Capacitive_Touch::on_deactivate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");
    publisher_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Capacitive_Touch::on_cleanup(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    publisher_.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Capacitive_Touch::on_shutdown(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return CallbackReturn::SUCCESS;
}

void
Capacitive_Touch::init_parameters()
{
    get_parameter("thresh", thresh_);
    get_parameter("filter_i", filter_size_);
    
    std::string msg_regex_;
    get_parameter("msg_regex", msg_regex_);
    regex_ = std::regex(msg_regex_);
}

void
Capacitive_Touch::init_pub_sub()
{
    publisher_ = this->create_publisher<sgd_msgs::msg::Touch>("handle_touch", default_qos);

    timer_ = this->create_wall_timer(10ms, std::bind(&Capacitive_Touch::read_serial, this));

    RCLCPP_DEBUG(get_logger(), "Initialised publisher on topic %s", 'handle_touch');
}

void
Capacitive_Touch::read_serial()
{
    if (serial.read_serial())
    {
        std::smatch matches;
        std::string msg = serial.get_msg();
        std::regex_search(msg, matches, regex_);
        
        int time, r, l;
        if (matches.size() > 3) // ist gut
        {
            time = std::stoi(matches[1]);
            r = std::stoi(matches[2]);
            l = std::stoi(matches[3]);
        } else { return; }

        // add r and l to filter
        filter_r.add_value(r);
        filter_l.add_value(l);

        sgd_msgs::msg::Touch touch_msg;
        touch_msg.header.stamp.sec = floor(time / 1000);
        int fr = filter_r.mov_avg();
        int fl = filter_l.mov_avg();
        touch_msg.has_detected = (fr > thresh_) && (fl > thresh_);
        touch_msg.value = fr < fl ? fr : fl;
        publisher_->publish(touch_msg);
    }
}

}   // namespace sgd_hardware_drivers

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_hardware_drivers::Capacitive_Touch>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
