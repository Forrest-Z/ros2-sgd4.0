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

#include "gnss/gnss_node.hpp"

namespace sgd_hardware_drivers
{

std::mutex rtcmMutex;   // mutex for rtcm string
char rtcm[1000];        // rtcm data
int numbytes;           // number of rtcm bytes read
std::mutex ggaMutex;    // mutex for gga data
std::string gga;        // GNGGA/GPGGA nmea string to send to ntrip server

Gnss_Node::Gnss_Node():
    LifecycleNode("gnss_node")
{
    // logging parameters
    declare_parameter("log_dir", rclcpp::ParameterValue(".ros/log/"));
    declare_parameter("log_severity", rclcpp::ParameterValue("I"));

    // receiver port and parser settings
    declare_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    declare_parameter("parser_file", rclcpp::ParameterValue("/home/ipp/dev_ws/src/ros2-sgd4.0/sgd_hardware_driver/gps/params/nmea_0183.xml"));
    declare_parameter("parser_type", rclcpp::ParameterValue("nmea"));

    // ros2 publisher
    declare_parameter("publish_local_pose", rclcpp::ParameterValue(true));
    declare_parameter("publish_tf", rclcpp::ParameterValue(true));
    declare_parameter("local_pose_topic", rclcpp::ParameterValue("gps/local"));
    declare_parameter("gps_sim_topic", rclcpp::ParameterValue("gps/sim"));  // subscriber topic for simulation
    declare_parameter("utc_topic", rclcpp::ParameterValue("clock/utc"));
    declare_parameter("gps_topic", rclcpp::ParameterValue("gps"));

    // transforms
    declare_parameter("transform_to_base_link", rclcpp::ParameterValue(true));
    declare_parameter("base_link_frame_id", rclcpp::ParameterValue("base_link"));
    declare_parameter("odom_frame_id", rclcpp::ParameterValue("odom"));

    // ntrip options
    declare_parameter("ntrip_server", rclcpp::ParameterValue(""));    // if server is empty do not use ntrip client
    declare_parameter("ntrip_port", rclcpp::ParameterValue(""));
    declare_parameter("ntrip_mountpoint", rclcpp::ParameterValue(""));
    declare_parameter("ntrip_auth", rclcpp::ParameterValue(""));
    declare_parameter("ntrip_send_nmea", rclcpp::ParameterValue(true));

    // initialize logging
    std::string log_dir_, log_sev_;
    get_parameter("log_dir", log_dir_);
    get_parameter("log_severity", log_sev_);
    std::string log_file(log_dir_ + "/gnss_node.csv");
    plog::init(plog::severityFromString(log_sev_.c_str()), log_file.c_str());
    PLOGI.printf("Created Gnss node. PLOG logging severity is %s", log_sev_.c_str());
}

Gnss_Node::~Gnss_Node() {}

CallbackReturn
Gnss_Node::on_configure(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Configuring...";
    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    if (is_pub_local_pose_)
    {
        init_transforms();
    }

    if (is_sim_) return CallbackReturn::SUCCESS;

    // initialize receiver serial port
    std::string port;       // port name e.g. /dev/serial/by-id/...
    get_parameter("port", port);
    try
    {
        // set start / stop frames and open port
        PLOGI.printf("Open port %s, set start frame to %c and end frame to %c", port.c_str(), '$', "\\n");
        serial.set_start_frame('$');
        serial.set_stop_frame('\n');
        serial.open_port(port, 115200);
    }
    catch(const std::exception& e)
    {
        PLOGE << e.what();
        RCLCPP_ERROR(get_logger(), e.what());
        return CallbackReturn::FAILURE;
    }
    
    // Initialize nmea parser
    if (parser_type_ == "nmea")
    {
        PLOGI << "Initialize parser for nmea data";
        parser_ = std::make_unique<Nmea_Parser>();
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Gnss_Node::on_activate(const rclcpp_lifecycle::State & state __attribute__((unused))) 
{
    PLOGI << "Activating...";
    init_pub_sub();

    if (is_sim_)
    {
        PLOGD << "Message: utc; lat; lon; x; y; hdop; fix";
        return CallbackReturn::SUCCESS;
    }

    // import xml file with nmea specification
    PLOGD.printf("Import parser file %s", xml_file_.c_str());
    parser_->import_xml(xml_file_);

    if (parser_->has_error())
    {
        PLOGE << parser_->get_last_error().to_string();
        RCLCPP_ERROR(get_logger(), parser_->get_last_error().to_string());
        return CallbackReturn::FAILURE;
    }

    // init hdop and fix status (debugging -> remove later)
    hdop = 100.0;
    status = 0;

    // wait for gnss fix
    int i = 0;
    while (parser_->fix() < 1 && i < 600 && rclcpp::ok())
    {
        if (serial.read_serial())   // read serial port
        {
            std::string msg = "$" + serial.get_msg() + "\n";
            parser_->parse_msg(msg);        // parse message to get fix status
        }
        if (i % 10 == 0)
        {
            // print out a warning every second
            RCLCPP_WARN(get_logger(), "Waiting for GNSS fix...");
        }
        rclcpp::sleep_for(100ms);
        i++;
    }
    // if (i >= 600 || ntrip_options_.server.empty())
    // {
    //     RCLCPP_WARN(get_logger(), "Could not get fix within 60 seconds. Start gnss without ntrip client.");
    //     return CallbackReturn::SUCCESS;
    // }
    
    PLOGI << "Start ntrip client";
    RCLCPP_INFO(get_logger(), "Start ntrip client");
    // initialize ntrip client and start in new thread
    client = std::make_unique<Ntrip_Client>(ntrip_options_);
    client->start_client();

    PLOGD << "Message: utc; lat; lon; x; y; hdop; fix";
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Gnss_Node::on_deactivate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGI << "Deactivating...";
    pub_navsatfix_->on_deactivate();
    if (is_pub_local_pose_)
    {
        pub_local_pose_->on_deactivate();
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn
Gnss_Node::on_cleanup(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGI << "Cleaning up...";
    pub_navsatfix_.reset();
    if (is_pub_local_pose_) pub_local_pose_.reset();

    client.release();   // stop ntrip client
    return CallbackReturn::SUCCESS;
}

CallbackReturn
Gnss_Node::on_shutdown(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGI << "Shutting down...";
    return CallbackReturn::SUCCESS;
}

void
Gnss_Node::init_parameters()
{
    get_parameter("use_sim_time", is_sim_);
    get_parameter("parser_file", xml_file_);
    get_parameter("parser_type", parser_type_);
    get_parameter("publish_local_pose", is_pub_local_pose_);
    get_parameter("publish_tf", is_publish_tf_);

    // topics
    get_parameter("local_pose_topic", local_pose_topic_);
    get_parameter("gps_sim_topic", gnss_sim_topic_);
    get_parameter("utc_topic", utc_clock_topic_);
    get_parameter("gps_topic", gnss_topic_);
    
    get_parameter("transform_to_base_link", is_tf_to_base_link_);
    get_parameter("base_link_frame_id", base_link_frame_id_);
    get_parameter("odom_frame_id", odom_frame_id_);

    // ntrip client parameter
    get_parameter("ntrip_server", ntrip_options_.server);
    get_parameter("ntrip_port", ntrip_options_.port);
    get_parameter("ntrip_mountpoint", ntrip_options_.mountpnt);
    get_parameter("ntrip_auth", ntrip_options_.auth);
    get_parameter("ntrip_send_nmea", ntrip_options_.nmea);
}

void
Gnss_Node::init_pub_sub()
{
    PLOGD << "Init publisher and subscriber";
    if (!is_sim_)
    {
        timer_ = this->create_wall_timer(10ms, std::bind(&Gnss_Node::read_serial, this));
        PLOGD.printf("Create publisher on topic %s", utc_clock_topic_.c_str());
        pub_utc_time_ = this->create_publisher<builtin_interfaces::msg::Time>(utc_clock_topic_, default_qos);
        pub_utc_time_->on_activate();
    }
    else
    {
        PLOGD.printf("Create subscription for topic %s", gnss_sim_topic_.c_str());
        sub_gps_sim = this->create_subscription<sensor_msgs::msg::NavSatFix>(gnss_sim_topic_, default_qos,
                        std::bind(&Gnss_Node::on_gps_sim_received, this, std::placeholders::_1));
    }

    if (is_publish_tf_)
    {
        // TODO bind to received positions
        PLOGD << "Create timer to publish tf every 100ms";
        timer_tf_ = this->create_wall_timer(100ms, std::bind(&Gnss_Node::publish_tf, this));
    }
    
    PLOGD.printf("Create publisher on topic %s", gnss_topic_.c_str());
    pub_navsatfix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(gnss_topic_, default_qos);
    pub_navsatfix_->on_activate();

    if (is_pub_local_pose_)
    {
        // create publisher for pose in local coordinates
        PLOGD.printf("Create publisher on topic %s", local_pose_topic_.c_str());
        pub_local_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(local_pose_topic_, default_qos);
        pub_local_pose_->on_activate();
    }
}

void
Gnss_Node::read_serial()
{
    while (serial.read_serial())
    {
        std::string msg = "$" + serial.get_msg() + "\n";
        parser_->parse_msg(msg);

        if (msg.find("GGA") < 5 && parser_->fix() > 0)    // if message starts with $GNGGA or $GPGGA
        {
            // save to gga
            ggaMutex.lock();
            gga = msg;
            ggaMutex.unlock();
        }
        
        // wait for all nmea messages -> TODO: make time based
        if (parser_->msg_complete())
        {
            // publish utc time
            builtin_interfaces::msg::Time utc;
            utc.sec = parser_->gps_time();
            utc.nanosec = (parser_->gps_time() - floor(parser_->gps_time())) * 1E9;
            pub_utc_time_->publish(utc);
            std::string debug_msg = std::to_string(parser_->gps_time()) + ";";
            debug_msg += std::to_string(now().nanoseconds()/1E6) + ";";

            // Create NavSatFix message and publish data
            sensor_msgs::msg::NavSatFix nsf;
            nsf.latitude = parser_->latitude();
            nsf.longitude = parser_->longitude();
            nsf.header.stamp = now();
            nsf.header.frame_id = "earth";

            auto val = parser_->get_data("hdop");
            if (val.second == 1)
            {
                double hdop_ = std::get<double>(val.first);

                if (hdop != hdop_ || status != parser_->fix())
                {
                    std::cout << "hdop: " << hdop << ", fix: " << parser_->fix() << "\n";
                    hdop = hdop_;
                    status = parser_->fix();
                }
            }

            nsf.status.status = (parser_->fix() > 1 ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                        : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX);

            // publish message and transforms
            debug_msg += std::to_string(nsf.latitude) + ";" + std::to_string(nsf.longitude) + ";";
            pub_navsatfix_->publish(nsf);
            
            if (is_pub_local_pose_)
            {
                auto local = to_local(nsf);
                debug_msg += std::to_string(local.pose.pose.position.x) + ";" + std::to_string(local.pose.pose.position.y);
                pub_local_pose_->publish(local);
            }
            PLOGD << debug_msg << ";" << std::get<double>(val.first) << ";" << parser_->fix();

            // Clear old message
            parser_->clear();
            last_msg_ = nsf;
            
        }
    }

    rtcmMutex.lock();
    if (numbytes > 0)
    {
        // send to receiver
        serial.write_serial(rtcm, numbytes);
        numbytes = 0;
    }
    rtcmMutex.unlock();
}

void
Gnss_Node::on_gps_sim_received(sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    pub_navsatfix_->publish(*msg);
    geometry_msgs::msg::PoseWithCovarianceStamped local_pose;
    if (pub_local_pose_)
    {
        local_pose = to_local(*msg);
        pub_local_pose_->publish(local_pose);
    }
    PLOGD.printf("%d; %.7f; %.7f; %.3f; %.3f; - ; %d",
            msg->header.stamp.sec, msg->latitude, msg->longitude, local_pose.pose.pose.position.x, local_pose.pose.pose.position.y, msg->status.status);

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
Gnss_Node::publish_tf()
{
    auto pose = to_local(last_msg_);
    geometry_msgs::msg::TransformStamped t;

    tf2::TimePoint transform_expiration = tf2_ros::fromMsg(last_msg_.header.stamp) +
        transform_tolerance_;
    t.header.stamp = tf2_ros::toMsg(transform_expiration);

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

    tf_broadcaster_->sendTransform(t);
}

void
Gnss_Node::init_transforms()
{
    PLOGD << "Init transforms...";
    //transform_tolerance_ = tf2::durationFromSec(1.0);
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
    //auto tmp_tf_base_gps_ = get_transform(base_link_frame_id_, "gps_link");
    tf_base_gps_ = get_transform(base_link_frame_id_, "gps_link").transform;
    PLOGD.printf("Transform from base_frame to gps_frame is %.3f, %.3f", tf_base_gps_.translation.x, tf_base_gps_.translation.y);
    PLOGD.printf("Set map origin to %.7f, %.7f", tf_.transform.translation.y, tf_.transform.translation.x);
    map_origin.set_global_coordinates(tf_.transform.translation.y, tf_.transform.translation.x);
}

geometry_msgs::msg::TransformStamped
Gnss_Node::get_transform(std::string target_frame, std::string source_frame)
{
    // Wait for transform to be available
    std::string err;
    int retries = 0;
    while (rclcpp::ok() && !tf_buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.1), &err)
        && retries < 10)
    {
        RCLCPP_DEBUG(get_logger(), "Timeout waiting for transform. Tf error: %s", err.c_str());
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
    auto tf = tf_buffer_->lookupTransform(target_frame, source_frame, rclcpp::Time(0), rclcpp::Duration(5,0));
    return tf;
}

geometry_msgs::msg::PoseWithCovarianceStamped
Gnss_Node::to_local(sensor_msgs::msg::NavSatFix msg)
{
    // create pose stamped msg
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = msg.header.stamp;
    pose.header.frame_id = "map";

    // wgs84 -> map frame
    sgd_util::LatLon ll(msg.latitude, msg.longitude);
    auto xy = ll.to_local(map_origin);

    // orientation of robot
    tf2::Quaternion q;
    auto tf_odom_ = get_transform("odom", "base_link");
    auto rot_z_ = tf2::getYaw(tf_odom_.transform.rotation);

    // publish local pose
    pose.pose.pose.position.x = xy.first - tf_base_gps_.translation.x * cos(rot_z_);
    pose.pose.pose.position.y = xy.second - tf_base_gps_.translation.x * sin(rot_z_);
    // pose.pose.pose.position.x = 247.37;
    // pose.pose.pose.position.y = 319.88;

    for (uint8_t i = 0; i < 36; i+=7)
    {
        // set diagonal of cov matrix
        pose.pose.covariance[i] = 1.2;
    }
    return pose;
}

}   // namespace sgd_hardware_drivers

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_hardware_drivers::Gnss_Node>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
