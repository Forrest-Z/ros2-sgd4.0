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

#include "uwb/uwb_node.hpp"

namespace sgd_hardware_drivers
{

UWB_Node::UWB_Node():
    rclcpp_lifecycle::LifecycleNode("uwb_node")
{
    // declare parameters with default values
    declare_parameter("log_dir", rclcpp::ParameterValue(".ros/log/"));
    declare_parameter("log_severity", rclcpp::ParameterValue("I"));

    declare_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    declare_parameter("tag_defs_file", rclcpp::ParameterValue("uwb_tag_defs.yaml"));
    declare_parameter("publish_wgs84_pose", rclcpp::ParameterValue(true));
    declare_parameter("publish_marker", rclcpp::ParameterValue(true));
    declare_parameter("exp_meas_frequ", rclcpp::ParameterValue(10));

    declare_parameter("local_pose_topic", rclcpp::ParameterValue("uwb/local"));
    declare_parameter("marker_topic", rclcpp::ParameterValue("uwb/tags"));
    declare_parameter("distance_topic", rclcpp::ParameterValue("uwb/distance"));
    declare_parameter("global_pose_topic", rclcpp::ParameterValue("uwb/global"));
    declare_parameter("odom_improved_topic", rclcpp::ParameterValue("odom/improved"));

    // optimizer = std::make_unique<LevMarq>();

    // initialize logging
    std::string log_sev_ = get_parameter("log_severity").as_string();
    std::string log_file(get_parameter("log_dir").as_string() + "/uwb_node.csv");
    plog::init(plog::severityFromString(log_sev_.c_str()), log_file.c_str());
    PLOGI.printf("Created uwb node. PLOG logging severity is %s", log_sev_.c_str());
    RCLCPP_INFO(get_logger(), "Created uwb node. Save log file to %s", log_file.c_str());
}

UWB_Node::~UWB_Node()
{
    // Destroy
}

CallbackReturn
UWB_Node::on_configure(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Configuring...";
    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    init_transforms();

    if (is_sim_)    return CallbackReturn::SUCCESS;

    std::string port;
    get_parameter("port", port);
    try
    {
        RCLCPP_INFO(get_logger(), "Open serial port %s", port.c_str());
        PLOGI.printf("Open serial port %s", port.c_str());
        serial.set_start_frame('{');
        serial.set_stop_frame('\n');
        serial.open_port(port, 115200);
    }
    catch(const sgd_io::io_exception& e)
    {
        RCLCPP_ERROR(get_logger(), e.what());
        PLOGE << e.what();
        return CallbackReturn::FAILURE;
    }
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn
UWB_Node::on_activate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Initializing...";
    init_pub_sub();

    pub_position_->on_activate();
    if (is_pub_wgs84_pose_) pub_wgs84_pose_->on_activate();
    if (is_pub_visual_)
    {
        pub_tag_marker_->on_activate();
        pub_dist_marker_->on_activate();
    }
    return init_yaml();
}

CallbackReturn
UWB_Node::on_deactivate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Deactivating...";
    pub_position_->on_deactivate();
    if (is_pub_wgs84_pose_) pub_wgs84_pose_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
UWB_Node::on_cleanup(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Cleaning up...";
    pub_position_.reset();
    if (is_pub_wgs84_pose_) pub_wgs84_pose_.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
UWB_Node::on_shutdown(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    PLOGD << "Shutting down...";
    return CallbackReturn::SUCCESS;
}

void
UWB_Node::init_parameters()
{
    get_parameter("use_sim_time", is_sim_);

    // get_parameter("tag_defs_file", tag_defs_file_);
    get_parameter("publish_local_pose", is_pub_wgs84_pose_);
    get_parameter("publish_marker", is_pub_visual_);
    // get_parameter("exp_meas_frequ", exp_frequ_);
}

void
UWB_Node::init_pub_sub()
{
    std::string local_pose_topic_ = get_parameter("local_pose_topic").as_string();
    PLOGD.printf("Create Publisher on topic '%s'", local_pose_topic_.c_str());
    pub_position_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(local_pose_topic_, default_qos);

    std::string odom_improved_topic_ = get_parameter("odom_improved_topic").as_string();
    PLOGD.printf("Create Publisher on topic '%s'", odom_improved_topic_.c_str());
    sub_odom_impr = this->create_subscription<nav_msgs::msg::Odometry>(odom_improved_topic_, default_qos,
            std::bind(&UWB_Node::on_odom_impr_received, this, std::placeholders::_1));

    if (is_pub_visual_)
    {
        std::string marker_topic_ = get_parameter("marker_topic").as_string();
        PLOGD.printf("Create Publisher on topic '%s'", marker_topic_.c_str());
        pub_tag_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, rclcpp::QoS(rclcpp::ParametersQoS()));

        std::string distance_topic_ = get_parameter("distance_topic").as_string();
        PLOGD.printf("Create Publisher on topic '%s'", distance_topic_.c_str());
        pub_dist_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(distance_topic_, rclcpp::QoS(rclcpp::ParametersQoS()));

        if (is_sim_)
        {
            // create subscriber for ground truth
            sub_ground_truth_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("ground_truth", default_qos,
                    std::bind(&UWB_Node::on_ground_truth_received, this, std::placeholders::_1));

            distribution = std::normal_distribution(0.0, 1.0);
        }
    }
    
    if (is_pub_wgs84_pose_)
    {
        std::string global_pose_topic_ = get_parameter("global_pose_topic").as_string();
        PLOGD.printf("Create Publisher on topic '%s'", global_pose_topic_.c_str());
        pub_wgs84_pose_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(global_pose_topic_, default_qos);
    }

    if (!is_sim_)
    {
        timer_ = this->create_wall_timer(5ms, std::bind(&UWB_Node::read_serial, this));
    }
    // timer_marker_ = this->create_wall_timer(30s, std::bind(&UWB_Node::publish_tag_position, this));
}

CallbackReturn
UWB_Node::init_yaml()
{
    YAML::Node doc = YAML::LoadFile(get_parameter("tag_defs_file").as_string());
    if (doc["anchors"])
    {
        for (std::size_t i = 0; i < doc["anchors"].size(); i++)
        {
            try
            {
                auto nd = doc["anchors"][i];
                if (nd["id"] && nd["pos"])
                {
                    auto tag_id = nd["id"].as<SGD_UWB_BEACON_ID_TYPE>();
                    auto vec = nd["pos"].as<std::vector<double>>();

                    // compute position in earth frame
                    // auto ll = sgd_util::LatLon(vec[0], vec[1]);
                    // auto xy = ll.to_local(map_origin);
                    
                    // compute position in local coordinates
                    point p;
                    p.x = vec[0];
                    p.y = vec[1];

                    tags.insert({tag_id, p});

                    uwb_pf.registerBeacon(p.x, p.y, tag_id);
                    
                    PLOGI.printf("Insert tag with id %d at position %.3f, %.3f", tag_id, p.x, p.y);
                }
            }
            catch(const YAML::Exception& e)
            {
                RCLCPP_ERROR(get_logger(), "YAML parse error: %s", e.what());
                PLOGE.printf("YAML parse error: %s", e.what());
            }
        }
        publish_tag_position();
    }

    RCLCPP_INFO(get_logger(), "Read %i tag definitions.", tags.size());
    PLOGI.printf("Read %i tag definitions.", tags.size());
    return (tags.size() > 0 ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE);
}

void
UWB_Node::read_serial()
{
    while (serial.read_serial())
    {
        // json string parsing
        std::string msg = "{" + serial.get_msg();
        PLOGV.printf("json: %s", msg.c_str());

        nlohmann::json js;
        try
        {
            js = nlohmann::json::parse(msg);
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(get_logger(), "JSON error: %s", e.what());
            PLOGW.printf("JSON error: %s", e.what());
            return;
        }
        
        SGD_UWB_BEACON_ID_TYPE tag_id;
        double dist;
        // example msg: {'anch': 16661, 'dist': '0A262C4277A9E33F', 'type': 'rng', 'src': 21505}
        if (js.count("anch") && js.count("dist"))
        {
            // PLOGD << msg;
            // message is from anchor
            tag_id = js["anch"].get<SGD_UWB_BEACON_ID_TYPE>();
            dist = js["dist"].get<int>();

            uwb_pf.updateUwbMeasurement(dist/1000.0, tag_id);
            PLOGD << tag_id << ";" << dist;
            publish_tag_radius(tag_id, dist);
        }
        //t_last_meas = now().nanoseconds() / 1.0E6;
    }
}

void
UWB_Node::publish_tag_position()
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "sgd";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.05;
    marker.lifetime.sec = 0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.8;
    marker.color.b = 0.0;

    for (auto tag : tags)
    {
        geometry_msgs::msg::Point p;
        p.x = tag.second.x;
        p.y = tag.second.y;
        marker.points.push_back(p);

        std_msgs::msg::ColorRGBA rgba;
        rgba.r = 0.0;
        rgba.g = 0.8;
        rgba.b = 0.0;
        rgba.a = 1.0;
        marker.colors.push_back(rgba);
    }
    pub_tag_marker_->publish(marker);
}

void
UWB_Node::publish_tag_radius(int tag_id, double dist)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "sgd";
    marker.id = tag_id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.lifetime.sec = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.765;
    marker.color.b = 1.0;

    // add points
    if (tags.count(tag_id))
    {
        auto xy = tags.at(tag_id);
        int num_pnts = 50;
        for (int i = 0; i <= num_pnts; i++)
        {
            geometry_msgs::msg::Point pnt;
            pnt.x = xy.x + sin(2*M_PI / num_pnts*i) * dist;
            pnt.y = xy.y + cos(2*M_PI / num_pnts*i) * dist;
            marker.points.push_back(pnt);
        }

        pub_dist_marker_->publish(marker);
    }
}

void
UWB_Node::on_odom_impr_received(nav_msgs::msg::Odometry::SharedPtr msg)
{
    // get last odometry message and calculate dx and dy
    // double dx = last_odom_.pose.pose.position.x - msg->pose.pose.position.x;
    // double dy = last_odom_.pose.pose.position.y - msg->pose.pose.position.y;
    // last_odom_ = *msg;

    uwb_pf.updateOdometry(msg->pose.pose.position.x, msg->pose.pose.position.y);
    
    if (uwb_pf.hasSolution())
    {
        publish_pose(uwb_pf.x(), uwb_pf.y());
    }
}

void
UWB_Node::publish_pose(double x, double y)
{
    // is_last_estimate_valid = true;
    
    // publish position
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = now();

    // orientation of robot
    tf2::Quaternion q;
    auto tf_odom_ = get_transform("odom", "base_link");
    auto rot_z_ = tf2::getYaw(tf_odom_.transform.rotation);

    // publish local pose
    pose.pose.pose.position.x = x - tf_base_uwb_.translation.x * cos(rot_z_);
    pose.pose.pose.position.y = y - tf_base_uwb_.translation.x * sin(rot_z_);
    //PLOGD << "Publish local pose: " << pose.pose.pose.position.x << ", " << pose.pose.pose.position.y;
    PLOGD << "0;" << pose.pose.pose.position.x << ";" << pose.pose.pose.position.y;

    pose.pose.pose.position.z = 0.1;
    pose.pose.pose.orientation.w = 1.0;
    pose.pose.pose.orientation.x = 0.0;
    pose.pose.pose.orientation.y = 0.0;
    pose.pose.pose.orientation.z = 0.0;

    // Covariance -> 6x6 matrix
    for (uint8_t i = 0; i < 36; i+=7)
    {
        // set diagonal to variance
        pose.pose.covariance[i] = std::pow(uwb_pf.meanError(), 2);
    }
    
    pub_position_->publish(pose);
}

void
UWB_Node::init_transforms() {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // transformation from earth -> map in WGS84 coordinates
    // according to REP-105 the x-axis points east (lon) and the y-axis north (lat)
    auto tf_ = get_transform("earth", "map");
    auto tmp_tf_base_gps_ = get_transform("base_link", "uwb_link");
    tf_base_uwb_ = tmp_tf_base_gps_.transform;
    map_origin.set_global_coordinates(tf_.transform.translation.y, tf_.transform.translation.x);
}

geometry_msgs::msg::TransformStamped
UWB_Node::get_transform(std::string target_frame, std::string source_frame)
{
    // Wait for transform to be available
    std::string err;
    int retries = 0;
    while (rclcpp::ok() && !tf_buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.1), &err)
        && retries < 10)
    {
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
    auto tf_tmp_ = tf_buffer_->lookupTransform(target_frame, source_frame, rclcpp::Time(0), rclcpp::Duration(5,0));
    return tf_tmp_;
}

void
UWB_Node::on_ground_truth_received(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    // if rate > 10 Hz -> skip message
    sgd_util::LatLon pos(msg->latitude, msg->longitude);
    auto local = pos.to_local(map_origin);
    // define some variables
    double max_distance_ = 50.0;
    double random_mult_ = 100.0;
    double random_add_ = 15.0;

    // calculate distance from tags to robo -> publish markers
    for (auto tag : tags)
    {
        // calculate euclidian distance from tag to robo
        double distance = sqrt(pow(tag.second.x - local.first,2) + pow(tag.second.y - local.second,2));
        if (distance > max_distance_)   continue;

        // add some noise
        double noise = distribution(generator);
        double dist_noise = distance * (1 + noise / 100.0);

        // generate random number -> if number is smaller than distance, skip publishing
        double random = (double)std::rand() / RAND_MAX;
        random = random * random_mult_ + random_add_;
        if (random > distance)
        {
            uwb_pf.updateUwbMeasurement(dist_noise, tag.first);
            // publish distance markers
            publish_tag_radius(tag.first, dist_noise);
        }
    }
}

}   // namespace sgd_hardware_drivers

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_hardware_drivers::UWB_Node>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
