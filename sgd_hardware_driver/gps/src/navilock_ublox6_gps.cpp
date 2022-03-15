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

namespace sgd_hardware_drivers
{

Navilock_UBlox6_GPS::Navilock_UBlox6_GPS():
    LifecycleNode("navilock_ublox6_gps")
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    declare_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    declare_parameter("xml_file", rclcpp::ParameterValue("/home/ipp/dev_ws/src/ros2-sgd4.0/sgd_hardware_driver/gps/params/nmea_0183.xml"));
    declare_parameter("parser_type", rclcpp::ParameterValue("ubx"));
    declare_parameter("publish_local_pose", rclcpp::ParameterValue(true));
}

Navilock_UBlox6_GPS::~Navilock_UBlox6_GPS()
{
    // Destroy
}

CallbackReturn
Navilock_UBlox6_GPS::on_configure(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();

    std::string port;
    get_parameter("port", port);
    serial.open_port(port, 115200);

    if (is_pub_local_pose_)
    {
        init_transforms();
    }
    init_pub_sub();

    // Initialize parser
    if (parser_type_ == "nmea")
    {
        parser_ = std::make_unique<Nmea_Parser>();
    }
    else
    {
        parser_ = std::make_unique<Ubx_Parser>();
    }
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Navilock_UBlox6_GPS::on_activate(const rclcpp_lifecycle::State & state __attribute__((unused))) 
{
    RCLCPP_DEBUG(get_logger(), "Activating");

    parser_->import_xml(xml_file_);

    if (parser_->has_error())
    {
        RCLCPP_ERROR(get_logger(), parser_->get_last_error().to_string());
        return CallbackReturn::FAILURE;
    }

    pub_navsatfix_->on_activate();
    if (is_pub_local_pose_)
    {
        pub_local_pose_->on_activate();
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Navilock_UBlox6_GPS::on_deactivate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");
    pub_navsatfix_->on_deactivate();
    if (is_pub_local_pose_)
    {
        pub_local_pose_->on_deactivate();
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Navilock_UBlox6_GPS::on_cleanup(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    pub_navsatfix_.reset();
    if (is_pub_local_pose_) pub_local_pose_.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Navilock_UBlox6_GPS::on_shutdown(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");

    return CallbackReturn::SUCCESS;
}

void
Navilock_UBlox6_GPS::init_parameters()
{
    get_parameter("xml_file", xml_file_);
    get_parameter("parser_type", parser_type_);
    get_parameter("publish_local_pose", is_pub_local_pose_);
}

void
Navilock_UBlox6_GPS::init_pub_sub()
{
    RCLCPP_DEBUG(get_logger(), "Init pub sub");
    timer_ = this->create_wall_timer(10ms, std::bind(&Navilock_UBlox6_GPS::read_serial, this));

    std::string log_("Initialised publisher on topic 'gps'");
    pub_navsatfix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", default_qos);
    if (is_pub_local_pose_)
    {
        pub_local_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("gps_local", default_qos);
        log_.append(" and on topic 'gps_local'.");
    }

    RCLCPP_DEBUG(get_logger(), "Initialised publisher on topic 'gps' and on topic 'gps_local'.");
}

void
Navilock_UBlox6_GPS::read_serial()
{
    if (serial.read_serial())
    {
        std::string msg = serial.get_msg();
        parser_->parse_msg(msg);

        // wait for all nmea messages
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
            pub_navsatfix_->publish(nsf);
            
            if (is_pub_local_pose_)
            {
                // create pose stamped msg
                geometry_msgs::msg::PoseWithCovarianceStamped pose;
                pose.header = nsf.header;

                // wgs84 -> map frame
                sgd_util::LatLon ll(parser_->latitude(), parser_->longitude());
                auto xy = ll.to_local(map_origin);

                // publish local pose
                pose.pose.pose.position.x = xy.first;
                pose.pose.pose.position.y = xy.second;

                pub_local_pose_->publish(pose);
            }

            // Clear old message
            parser_->clear();
        }
    }
}

void
Navilock_UBlox6_GPS::init_transforms() {
    auto tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    auto tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Wait for transform to be available
    std::string err;
    int retries = 0;
    while (rclcpp::ok() && !tf_buffer_->canTransform("earth", "map", tf2::TimePointZero, tf2::durationFromSec(0.1), &err)
        && retries < 10)
    {
        RCLCPP_INFO(this->get_logger(), "Timeout waiting for transform. Tf error: %s", err);
        err.clear();
        rclcpp::sleep_for(500000000ns);
        retries++;
    }

    if (retries > 9)
    {
        RCLCPP_ERROR(get_logger(), "Could not retrieve transform from earth to map.");
        return;
    }

    // transformation from earth -> map in WGS84 coordinates
    // according to REP-105 the x-axis points east (lon) and the y-axis north (lat)
    auto tf_ = tf_buffer_->lookupTransform("earth", "map", rclcpp::Time(0), rclcpp::Duration(5,0));
    map_origin.set_global_coordinates(tf_.transform.translation.y, tf_.transform.translation.x);
}

}   // namespace sgd_hardware_drivers

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_hardware_drivers::Navilock_UBlox6_GPS>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
