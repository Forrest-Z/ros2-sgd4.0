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
    declare_parameter("tag_defs", rclcpp::ParameterValue("uwb_tag_defs.yaml"));
    declare_parameter("publish_wgs84_pose", rclcpp::ParameterValue(true));
    declare_parameter("publish_marker", rclcpp::ParameterValue(true));
    declare_parameter("exp_meas_frequ", rclcpp::ParameterValue(10));

    optimizer = std::make_unique<LevMarq>();

    // initialize logging
    std::string log_dir_;
    get_parameter("log_dir", log_dir_);
    std::string log_sev_;
    get_parameter("log_severity", log_sev_);
    plog::init(plog::severityFromString(log_sev_.c_str()), (log_dir_ + "/" + sgd_util::create_log_file("uwb")).c_str());
    PLOGD << "tag_id; dist_m";
}

UWB_Node::~UWB_Node()
{
    // Destroy
}

CallbackReturn
UWB_Node::on_configure(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    // Initialize parameters, pub/sub, services, etc.
    init_parameters();
    std::string port;
    get_parameter("port", port);
    try
    {
        RCLCPP_INFO(get_logger(), "Open serial port %s", port.c_str());
        serial.set_start_frame('{');
        serial.set_stop_frame('\n');
        serial.open_port(port, 115200);
    }
    catch(const sgd_io::io_exception& e)
    {
        RCLCPP_ERROR(get_logger(), e.what());
        return CallbackReturn::FAILURE;
    }
    
    init_transforms();
    is_last_estimate_valid = false;

    return CallbackReturn::SUCCESS;
}

CallbackReturn
UWB_Node::on_activate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    init_pub_sub();

    pub_position_->on_activate();
    if (is_pub_wgs84_pose_) pub_wgs84_pose_->on_activate();
    if (is_pub_marker_)
    {
        pub_tag_marker_->on_activate();
        pub_dist_marker_->on_activate();
    }
    return init_yaml();
}

CallbackReturn
UWB_Node::on_deactivate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    pub_position_->on_deactivate();
    if (is_pub_wgs84_pose_) pub_wgs84_pose_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
UWB_Node::on_cleanup(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    pub_position_.reset();
    if (is_pub_wgs84_pose_) pub_wgs84_pose_.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
UWB_Node::on_shutdown(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    return CallbackReturn::SUCCESS;
}

void
UWB_Node::init_parameters()
{
    get_parameter("tag_defs", tag_definitions_);
    get_parameter("publish_local_pose", is_pub_wgs84_pose_);
    get_parameter("publish_marker", is_pub_marker_);
    get_parameter("exp_meas_frequ", exp_frequ_);
}

void
UWB_Node::init_pub_sub()
{
    RCLCPP_DEBUG(get_logger(), "Create Publisher on topic 'uwb/local'");
    pub_position_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("uwb/local", default_qos);

    sub_odom_impr = this->create_subscription<sgd_msgs::msg::OdomImproved>("odom/improved", default_qos,
            std::bind(&UWB_Node::on_odom_impr_received, this, std::placeholders::_1));

    if (is_pub_marker_)
    {
        RCLCPP_DEBUG(get_logger(), "Create Publisher on topic 'uwb/tags'");
        pub_tag_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("uwb/tags", default_qos);
        RCLCPP_DEBUG(get_logger(), "Create Publisher on topic 'uwb/distance'");
        pub_dist_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("uwb/distance", default_qos);
    }
    
    if (is_pub_wgs84_pose_)
    {
        RCLCPP_DEBUG(get_logger(), "Create Publisher on topic 'uwb/global'");
        pub_wgs84_pose_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("uwb/global", default_qos);
    }

    timer_ = this->create_wall_timer(5ms, std::bind(&UWB_Node::read_serial, this));
    timer_marker_ = this->create_wall_timer(30s, std::bind(&UWB_Node::publish_marker, this));
}

CallbackReturn
UWB_Node::init_yaml()
{
    num_tags = 0;
    YAML::Node doc = YAML::LoadFile(tag_definitions_);
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

                    // compute position in map frame
                    auto ll = sgd_util::LatLon(vec[0], vec[1]);
                    auto xy = ll.to_local(map_origin);
                    
                    // TODO : Remove to in order to switch to lat/long coordinates:
                    xy.first = vec[0];
                    xy.second = vec[1];

                    optimizer->tags.insert({tag_id, xy});

                    uwb_pf.registerBeacon(xy.first, xy.second, tag_id);
                    
                    RCLCPP_INFO(get_logger(), "Insert tag with id %d at position %.3f, %.3f",
                                tag_id, xy.first, xy.second);
                    num_tags++;
                }
            }
            catch(const YAML::Exception& e)
            {
                RCLCPP_ERROR(get_logger(), "YAML parse error: %s", e.what());
            }
        }
        publish_marker();
    }

    RCLCPP_INFO(get_logger(), "Read %i tag definitions.", optimizer->tags.size());
    return (num_tags > 0 ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE);
}

void
UWB_Node::read_serial()
{
    // compute condition for optimization
    // get time since last received measurement
    double t = now().nanoseconds() / 1.0E6; // current time in milliseconds
    // if ((t - t_last_meas) > 2000.0/exp_frequ_)
    // {
    //     if (optimizer->measuredRanges.size() > 2)   // at least 3 measurements are required to calculate a position
    //     {
    //         // if the last measurement was received more than 1/frequ ago
    //         auto pos = optimizer->estimatePose();
    //         publish_pose(pos.first, pos.second);
    //     }
    //     else if (is_last_estimate_valid)    // print warning only once
    //     {
    //         RCLCPP_WARN(get_logger(), "Not enough range measurements to calculate position.");
    //         is_last_estimate_valid = false;
    //     }
    // }

    while (serial.read_serial())
    {
        // json string parsing
        std::string msg = "{" + serial.get_msg();

        nlohmann::json js;
        try
        {
            js = nlohmann::json::parse(msg);
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(get_logger(), "JSON error: %s", e.what());
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

            //uwb_pf.updateUwbMeasurement(dist/1000.0, tag_id);
            PLOGD << tag_id << ";" << dist;
            //publish_dist_marker(tag_id, dist);
        }
        t_last_meas = now().nanoseconds() / 1.0E6;
    }
}

void
UWB_Node::publish_pose(double x, double y)
{
    is_last_estimate_valid = true;
    
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
UWB_Node::publish_marker()
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
    marker.lifetime.sec = 30;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.8;
    marker.color.b = 0.0;

    for (auto tag : optimizer->tags)
    {
        geometry_msgs::msg::Point p;
        p.x = tag.second.first;
        p.y = tag.second.second;
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
UWB_Node::publish_dist_marker(int tag_id, double dist)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "sgd";
    marker.id = tag_id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.lifetime.sec = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.8;
    marker.color.b = 0.0;

    // add points
    if (optimizer->tags.count(tag_id))
    {
        auto xy = optimizer->tags.at(tag_id);
        int num_pnts = 50;
        for (int i = 0; i <= num_pnts; i++)
        {
            geometry_msgs::msg::Point pnt;
            pnt.x = xy.first + sin(2*M_PI / num_pnts*i) * dist;
            pnt.y = xy.second + cos(2*M_PI / num_pnts*i) * dist;
            marker.points.push_back(pnt);
        }

        pub_dist_marker_->publish(marker);
    }

    
}

void
UWB_Node::on_odom_impr_received(sgd_msgs::msg::OdomImproved::SharedPtr msg)
{
    uwb_pf.updateOdometry(msg->dx, msg->dy);
    
    if (uwb_pf.hasSolution())
    {
        publish_pose(uwb_pf.x(), uwb_pf.y());
    }
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

}   // namespace sgd_hardware_drivers

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_hardware_drivers::UWB_Node>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
