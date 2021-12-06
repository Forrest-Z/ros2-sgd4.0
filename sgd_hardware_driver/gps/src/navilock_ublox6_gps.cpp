// Copyright 2021 HAW Hamburg
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

#include "gps/navilock_ublox6_gps.hpp"

namespace sgd_hardware
{

using namespace std::chrono_literals;   // if a timer is used

Navilock_UBlox6_GPS::Navilock_UBlox6_GPS():
    nav2_util::LifecycleNode("navilock_ublox6_gps", "", true)
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    add_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    add_parameter("xml_file", rclcpp::ParameterValue("/home/ipp/dev_ws/src/ros2-sgd4.0/sgd_hardware_driver/gps/params/nmea_0183.xml"));
    add_parameter("parser_type", rclcpp::ParameterValue("ubx"));
}

Navilock_UBlox6_GPS::~Navilock_UBlox6_GPS()
{
    // Destroy
}

nav2_util::CallbackReturn
Navilock_UBlox6_GPS::on_configure(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    init_pub_sub();

    // Initialize parser
    if (parser_type_ == "nmea")
    {
        parser_ = std::make_shared<Nmea_Parser>();
    }
    else
    {
        parser_ = std::make_shared<Ubx_Parser>();
    }
    
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Navilock_UBlox6_GPS::on_activate(const rclcpp_lifecycle::State & state __attribute__((unused))) 
{
    RCLCPP_DEBUG(get_logger(), "Activating");

    parser_->import_xml(xml_file_);

    if (parser_->has_error())
    {
        RCLCPP_ERROR(get_logger(), parser_->get_last_error().to_string());
        return nav2_util::CallbackReturn::FAILURE;
    }

    publisher_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Navilock_UBlox6_GPS::on_deactivate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");
    publisher_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Navilock_UBlox6_GPS::on_cleanup(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    publisher_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Navilock_UBlox6_GPS::on_shutdown(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");

    return nav2_util::CallbackReturn::SUCCESS;
}

void
Navilock_UBlox6_GPS::init_parameters()
{
    get_parameter("port", port_);
    get_parameter("xml_file", xml_file_);
    get_parameter("parser_type", parser_type_);
}

void
Navilock_UBlox6_GPS::init_pub_sub()
{
    RCLCPP_DEBUG(get_logger(), "Init pub sub");
    std::string serial_topic = "serial_" + port_.substr(port_.find_last_of("/")+1);

    publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", default_qos);
    subscriber_ = this->create_subscription<sgd_msgs::msg::Serial>(
        serial_topic, default_qos, std::bind(&Navilock_UBlox6_GPS::on_serial_received, this, std::placeholders::_1));

    RCLCPP_DEBUG(get_logger(), "Initialised publisher on topic %s and subscriber on topic %s.",
            "gps", serial_topic.c_str());
}

void
Navilock_UBlox6_GPS::on_serial_received(const sgd_msgs::msg::Serial::SharedPtr msg)
{
    std::string line = msg->msg;
    parser_->parse_msg(line);

    // TODO wait for all nmea messages
    if (parser_->msg_complete())
    {
        // Create NavSatFix message and publish data
        sensor_msgs::msg::NavSatFix nsf;
        nsf.latitude = parser_->latitude();
        nsf.longitude = parser_->longitude();
        nsf.header.stamp.sec = parser_->time();
        nsf.header.stamp.nanosec = (parser_->time() - floor(parser_->time())) * 1E9;

        auto val = parser_->get_data("hdop");
        double hdop = -1;
        if (val.second == 1)
        {
            hdop = std::get<double>(val.first);
            RCLCPP_INFO(get_logger(), "HDOP is %f", hdop);
        }

        nsf.status.status = (parser_->fix() > 1 ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                    : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX);

        // publish message and transforms
        publisher_->publish(nsf);
        
        // Clear old message
        parser_->clear();
    }
}

}   // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_hardware::Navilock_UBlox6_GPS>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
