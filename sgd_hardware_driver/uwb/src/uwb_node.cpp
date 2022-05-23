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
    declare_parameter("port", rclcpp::ParameterValue("/dev/novalue"));
    declare_parameter("tag_defs", rclcpp::ParameterValue("uwb_tag_defs.yaml"));
    declare_parameter("msg_regex", rclcpp::ParameterValue("\"anch\":(\\d+),\"dist\":\"([0-9A-F]+)\",\"type\":\"(\\w+)\",\"src\":(-?\\d+)"));
    declare_parameter("publish_wgs84_pose", rclcpp::ParameterValue(true));
    declare_parameter("exp_meas_frequ", rclcpp::ParameterValue(10));

    optimizer = std::make_unique<LevMarq>();
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
        RCLCPP_WARN(get_logger(), e.what());
    }
    
    RCLCPP_INFO(get_logger(), "Init transforms");
    init_transforms();

    init_pub_sub();
    is_last_estimate_valid = false;

    return CallbackReturn::SUCCESS;
}

CallbackReturn
UWB_Node::on_activate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Activating");
    pub_position_->on_activate();
    if (is_pub_wgs84_pose_) pub_wgs84_pose_->on_activate();

    // read yaml and add tags to optimizer
    auto rgx = std::regex("- \\[(0x\\d{4}), (\\d+\\.\\d+), (\\d+\\.\\d+)]");
    std::smatch matches;

    // set number of ranges and tags
    num_tags = 0;

    // read file
    std::ifstream file(tag_definitions_);
    std::string line; 
    while (getline(file, line))
    {
        std::regex_search(line, matches, rgx);
        if (matches.size() > 3) // ist gut
        {
            try {
                auto tag_id = std::stoi(matches[1], nullptr, 16);
                auto lat = std::stod(matches[2]);
                auto lon = std::stod(matches[3]);

                RCLCPP_INFO(get_logger(), "Insert tag with id %d lat/lon %f, %f", tag_id, lat, lon);

                // compute position in map frame
                auto ll = sgd_util::LatLon(lat, lon);
                optimizer->tags.insert({tag_id, ll.to_local(map_origin)});
                num_tags++;
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), e.what());
            }
        }
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn
UWB_Node::on_deactivate(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Deactivating");
    pub_position_->on_deactivate();
    if (is_pub_wgs84_pose_) pub_wgs84_pose_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
UWB_Node::on_cleanup(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Cleanup");
    pub_position_.reset();
    if (is_pub_wgs84_pose_) pub_wgs84_pose_.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn
UWB_Node::on_shutdown(const rclcpp_lifecycle::State & state __attribute__((unused)))
{
    RCLCPP_DEBUG(get_logger(), "Shutdown");
    return CallbackReturn::SUCCESS;
}

void
UWB_Node::init_parameters()
{   
    std::string msg_regex_;
    get_parameter("msg_regex", msg_regex_);
    try {
        regex_ = std::regex(msg_regex_);
    } catch (const std::regex_error& re)
    {
        RCLCPP_WARN(get_logger(), re.what());
    }

    get_parameter("tag_defs", tag_definitions_);
    get_parameter("publish_local_pose", is_pub_wgs84_pose_);
    get_parameter("exp_meas_frequ", exp_frequ_);
}

void
UWB_Node::init_pub_sub()
{
    RCLCPP_INFO(get_logger(), "Init publisher and subscriber");
    pub_position_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("uwb_local", default_qos);
    if (is_pub_wgs84_pose_)
    {
        pub_wgs84_pose_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("uwb", default_qos);
    }

    timer_ = this->create_wall_timer(10ms, std::bind(&UWB_Node::read_serial, this));

    RCLCPP_DEBUG(get_logger(), "Initialised publisher on topic %s", 'uwb_pos');
}

void
UWB_Node::read_serial()
{
    // compute condition for optimization
    // get time since last received measurement
    double t = now().nanoseconds() / 1.0E6; // current time in milliseconds
    if ((t - t_last_meas) > 2000.0/exp_frequ_)
    {
        if (optimizer->measuredRanges.size() > 2)   // at least 3 measurements are required to calculate a position
        {
            // if the last measurement was received more than 1/frequ ago
            auto pos = optimizer->estimatePose();
            publish_pose(pos.first, pos.second);
        }
        else if (is_last_estimate_valid)    // print warning only once
        {
            RCLCPP_WARN(get_logger(), "Not enough range measurements to calculate position.");
            is_last_estimate_valid = false;
        }
    }

    if (serial.read_serial())
    {
        std::smatch matches;
        std::string msg = serial.get_msg();
        std::regex_search(msg, matches, regex_);

        int tag_id;
        double dist;
        // example msg: {'anch': 16661, 'dist': '0A262C4277A9E33F', 'type': 'rng', 'src': 21505}
        if (matches.size() > 4) // ist gut
        {
            tag_id = std::stoi(matches[1]);

            dist = sgd_util::toDouble(matches[2].str(), true);
            //real_x = std::stoi(matches[3]) / 1000.0;
            //real_y = std::stoi(matches[4]) / 1000.0;
            //RCLCPP_INFO(get_logger(), "Received dist: %.3f", dist);
            
            // add range to optimizer
            if (optimizer->measuredRanges.find(tag_id) != optimizer->measuredRanges.end())
            {
                // measurement from this tag is already contained in map -> start optimization (map is cleared afterwards)
                auto pos = optimizer->estimatePose();
                publish_pose(pos.first, pos.second);
            }

            optimizer->measuredRanges.insert({tag_id, dist});

            // if the size of the map equalds the number of tags -> start optimization
            if (num_tags <= optimizer->measuredRanges.size())
            {
                auto pos = optimizer->estimatePose();
                publish_pose(pos.first, pos.second);
            }
        }
        t_last_meas = now().nanoseconds() / 1.0E6;
        is_last_estimate_valid = true;
    }
}

void
UWB_Node::publish_pose(double x, double y)
{
    //RCLCPP_INFO(get_logger(), "Computed position: %.5f %.5f\tOriginal: %.2f %.2f", x - 165.8265, y - 273.4229,
    //                            real_x, real_y);
    RCLCPP_INFO(get_logger(), "Pos: %.3f, %.3f", x, y);
    // publish position
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = now();
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.position.z = 0.1;
    pose.pose.pose.orientation.w = 1.0;
    pose.pose.pose.orientation.x = 0.0;
    pose.pose.pose.orientation.y = 0.0;
    pose.pose.pose.orientation.z = 0.0;
    pub_position_->publish(pose);
}

void
UWB_Node::init_transforms() {
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
    auto node = std::make_shared<sgd_hardware_drivers::UWB_Node>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
