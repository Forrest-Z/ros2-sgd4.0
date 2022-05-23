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
    //declare_parameter("use_sim_time", rclcpp::ParameterValue(true));
    declare_parameter("parser_file", rclcpp::ParameterValue("/home/ipp/dev_ws/src/ros2-sgd4.0/sgd_hardware_driver/gps/params/nmea_0183.xml"));
    declare_parameter("parser_type", rclcpp::ParameterValue("nmea"));
    declare_parameter("publish_local_pose", rclcpp::ParameterValue(true));
    declare_parameter("publish_tf", rclcpp::ParameterValue(true));
    declare_parameter("odom_frame_id", rclcpp::ParameterValue("odom"));
    declare_parameter("log_dir", rclcpp::ParameterValue(""));
}

Navilock_UBlox6_GPS::~Navilock_UBlox6_GPS()
{
    // Destroy
}

CallbackReturn
Navilock_UBlox6_GPS::on_configure(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_INFO(get_logger(), "Configuring");

    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    if (is_pub_local_pose_)
    {
        init_transforms();
    }

    if (is_sim_) return CallbackReturn::SUCCESS;
    std::string port;
    get_parameter("port", port);
    serial.set_start_frame('$');
    serial.set_stop_frame('\n');
    serial.open_port(port, 115200);

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
    RCLCPP_INFO(get_logger(), "Activating");
    init_pub_sub();

    if (is_sim_) return CallbackReturn::SUCCESS;

    // initialize parser if is_sim_ == false
    parser_->import_xml(xml_file_);

    if (parser_->has_error())
    {
        RCLCPP_ERROR(get_logger(), parser_->get_last_error().to_string());
        return CallbackReturn::FAILURE;
    }

    pub_utc_time_->on_activate();

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
    get_parameter("use_sim_time", is_sim_);
    get_parameter("parser_file", xml_file_);
    get_parameter("parser_type", parser_type_);
    get_parameter("publish_local_pose", is_pub_local_pose_);
    get_parameter("publish_tf", is_publish_tf_);
    get_parameter("odom_frame_id", odom_frame_id_);
    get_parameter("log_dir", ros_log_dir_);
}

void
Navilock_UBlox6_GPS::init_pub_sub()
{
    RCLCPP_DEBUG(get_logger(), "Init pub sub");
    if (!is_sim_)
    {
        timer_ = this->create_wall_timer(10ms, std::bind(&Navilock_UBlox6_GPS::read_serial, this));
        pub_utc_time_ = this->create_publisher<builtin_interfaces::msg::Time>("clock_utc", default_qos);
    }
    else
    {
        sub_gps_sim = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps_sim", default_qos,
                        std::bind(&Navilock_UBlox6_GPS::on_gps_sim_received, this, std::placeholders::_1));
    }

    if (is_publish_tf_)
    {
        timer_tf_ = this->create_wall_timer(100ms, std::bind(&Navilock_UBlox6_GPS::publish_tf, this));
    }
    
    std::string log_("Initialised publisher on topic 'gps'");
    pub_navsatfix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", default_qos);
    if (is_pub_local_pose_)
    {
        pub_local_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("gps_local", default_qos);
        log_.append(" and on topic 'gps_local'.");
    }

    // activate publisher
    pub_navsatfix_->on_activate();
    if (is_pub_local_pose_)
    {
        pub_local_pose_->on_activate();
    }

    RCLCPP_DEBUG(get_logger(), "Initialised publisher on topic 'gps' and on topic 'gps_local'.");
}

void
Navilock_UBlox6_GPS::read_serial()
{
    if (serial.read_serial())
    {
        std::string msg = "$" + serial.get_msg();
        parser_->parse_msg(msg);

        // wait for all nmea messages
        if (parser_->msg_complete())
        {
            // publish utc time
            builtin_interfaces::msg::Time utc;
            utc.sec = parser_->gps_time();
            utc.nanosec = (parser_->gps_time() - floor(parser_->gps_time())) * 1E9;
            pub_utc_time_->publish(utc);

            // Create NavSatFix message and publish data
            sensor_msgs::msg::NavSatFix nsf;
            nsf.latitude = parser_->latitude();
            nsf.longitude = parser_->longitude();
            nsf.header.stamp = now();
            nsf.header.frame_id = "earth";

            auto val = parser_->get_data("hdop");
            double hdop = -1;
            if (val.second == 1)
            {
                hdop = std::get<double>(val.first);
                //RCLCPP_INFO(get_logger(), "HDOP is %f", hdop);
            }

            nsf.status.status = (parser_->fix() > 1 ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                        : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX);

            // publish message and transforms
            pub_navsatfix_->publish(nsf);
            
            if (is_pub_local_pose_)
            {
                pub_local_pose_->publish(to_local(nsf));
            }

            // log to file

            // Clear old message
            parser_->clear();

            last_msg_ = nsf;
        }
    }
}

void
Navilock_UBlox6_GPS::on_gps_sim_received(sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    pub_navsatfix_->publish(*msg);
    if (pub_local_pose_)
    {
        pub_local_pose_->publish(to_local(*msg));
    }

    // calculate orientation before overwriting last_msg
    sgd_util::LatLon ll1(last_msg_.latitude, last_msg_.longitude);
    sgd_util::LatLon ll2(msg->latitude, msg->longitude);

    auto xy1 = ll1.to_local(map_origin);
    auto xy2 = ll2.to_local(map_origin);

    double dx = xy2.first - xy1.first;
    double dy = xy2.second - xy1.second;
    if ((abs(dy) > 0.05 || abs(dx) > 0.05) && abs(dx) < 10.0 && abs(dy) < 10)
    {
        // calculate heading if robot has moved enough
        heading_ = atan2(xy2.second - xy1.second, xy2.first - xy1.first);
    }
    
    last_msg_ = *msg;
}

void
Navilock_UBlox6_GPS::publish_tf()
{
    // TODO add orientation
    auto pose = to_local(last_msg_);
    geometry_msgs::msg::TransformStamped t;

    tf2::TimePoint transform_expiration = tf2_ros::fromMsg(last_msg_.header.stamp) +
        transform_tolerance_;
    t.header.stamp = tf2_ros::toMsg(transform_expiration);

    //t.header.stamp = last_msg_.header.stamp;
    t.header.frame_id = "map";
    t.child_frame_id = "odom";

    auto tf_odom_ = get_transform("odom", "base_link");

    // orientation of robot
    tf2::Quaternion q;
    auto rot_z_ = tf2::getYaw(tf_odom_.transform.rotation);
    double tf_base_gps_x_ = tf_base_gps_.translation.x * cos(rot_z_) + tf_base_gps_.translation.y * sin(rot_z_);
    double tf_base_gps_y_ = tf_base_gps_.translation.y * cos(rot_z_) + tf_base_gps_.translation.x * sin(rot_z_);

    t.transform.translation.x = pose.pose.pose.position.x - tf_odom_.transform.translation.x - tf_base_gps_x_;
    t.transform.translation.y = pose.pose.pose.position.y - tf_odom_.transform.translation.y - tf_base_gps_y_;
    t.transform.translation.z = 0.0;

    //RCLCPP_INFO(get_logger(), "Transform base -> gps: %.2f, %.2f", tf_base_gps_x_, tf_base_gps_y_);

    tf_broadcaster_->sendTransform(t);
}

void
Navilock_UBlox6_GPS::init_transforms()
{
    transform_tolerance_ = tf2::durationFromSec(1.0);
    // transformation from earth -> map in WGS84 coordinates
    // according to REP-105 the x-axis points east (lon) and the y-axis north (lat)
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (is_publish_tf_)
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    auto tf_ = get_transform("earth", "map");
    auto tmp_tf_base_gps_ = get_transform("base_link", "gps_link");
    tf_base_gps_ = tmp_tf_base_gps_.transform;
    map_origin.set_global_coordinates(tf_.transform.translation.y, tf_.transform.translation.x);
}

geometry_msgs::msg::TransformStamped
Navilock_UBlox6_GPS::get_transform(std::string target_frame, std::string source_frame)
{
    // Wait for transform to be available
    std::string err;
    int retries = 0;
    while (rclcpp::ok() && !tf_buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.1), &err)
        && retries < 10)
    {
        RCLCPP_INFO(this->get_logger(), "Timeout waiting for transform. Tf error: %s", err.c_str());
        err.clear();
        rclcpp::sleep_for(500000000ns);
        retries++;
    }

    if (retries > 9)
    {
        RCLCPP_ERROR(get_logger(), "Could not retrieve transform from %s to %s.", target_frame.c_str(), source_frame.c_str());
        geometry_msgs::msg::TransformStamped ts;
        return ts;
    }

    // transformation from earth -> map in WGS84 coordinates
    // according to REP-105 the x-axis points east (lon) and the y-axis north (lat)
    return tf_buffer_->lookupTransform(target_frame, source_frame, rclcpp::Time(0), rclcpp::Duration(5,0));
}

geometry_msgs::msg::PoseWithCovarianceStamped
Navilock_UBlox6_GPS::to_local(sensor_msgs::msg::NavSatFix msg)
{
    // create pose stamped msg
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = msg.header.stamp;
    pose.header.frame_id = "map";

    // wgs84 -> map frame
    sgd_util::LatLon ll(msg.latitude, msg.longitude);
    auto xy = ll.to_local(map_origin);

    // publish local pose
    pose.pose.pose.position.x = xy.first;
    pose.pose.pose.position.y = xy.second;
    return pose;
}

}   // namespace sgd_hardware_drivers

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_hardware_drivers::Navilock_UBlox6_GPS>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
