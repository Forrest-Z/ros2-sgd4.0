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

# include "sgd_util/visualizer.hpp"

namespace sgd_util
{

Visualizer::Visualizer() : rclcpp::Node("visualizer")
{
    RCLCPP_DEBUG(get_logger(), "Creating");

    declare_parameter("gnss_topic", rclcpp::ParameterValue("gnss/local"));
    declare_parameter("uwb_pose_topic", rclcpp::ParameterValue("uwb/local"));
    declare_parameter("uwb_marker_topic", rclcpp::ParameterValue("uwb/marker"));
    declare_parameter("odom_topic", rclcpp::ParameterValue("odom"));

    declare_parameter("marker_lifetime", rclcpp::ParameterValue(30));

    get_parameter("marker_lifetime", marker_lifetime_);

    // Initialize parameters, pub/sub, services, etc.
    init_pub_sub();

    // init colors for visualization
    status_colors[0] = create_color(135, 135, 135);
    status_colors[1] = create_color(0  , 255, 255);
    status_colors[2] = create_color(200, 0  , 0  );
    status_colors[3] = create_color(255, 255, 0  );
    status_colors[4] = create_color(0  , 200, 0  );
    status_colors[5] = create_color(255, 135, 0  );
}

Visualizer::~Visualizer()
{
    // Destroy
}

void
Visualizer::init_pub_sub()
{
    std::string gnss_topic_ = get_parameter("gnss_topic").as_string();
    if (!gnss_topic_.empty())
    {
        sub_gnss_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(gnss_topic_, default_qos,
                std::bind(&Visualizer::on_gnss_received, this, std::placeholders::_1));
        pub_gnss_pose_ = create_publisher<visualization_msgs::msg::Marker>("visual/gnss", default_qos);
    }

    std::string uwb_topic_ = get_parameter("uwb_pose_topic").as_string();
    if (!uwb_topic_.empty())
    {
        sub_uwb_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(uwb_topic_, default_qos,
                std::bind(&Visualizer::on_uwb_received, this, std::placeholders::_1));
        pub_uwb_pose_ = create_publisher<visualization_msgs::msg::Marker>("visual/uwb", default_qos);
    }

    std::string odom_topic_ = get_parameter("odom_topic").as_string();
    if (!odom_topic_.empty())
    {
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, default_qos,
                std::bind(&Visualizer::on_odom_received, this, std::placeholders::_1));
        pub_odom_orientation_ = create_publisher<geometry_msgs::msg::PoseStamped>("visual/odom", default_qos);
    }
}

void
Visualizer::on_gnss_received(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // publish gnss pose
    visualization_msgs::msg::Marker marker = create_marker();
    marker.header = msg->header;
    marker.pose = msg->pose.pose;
    
    // TODO set color depending on status
    int status = (int)std::round(msg->pose.covariance[0]);
    if (status > 5 || status < 0)
    {
        status = 0;
    }
    marker.color = status_colors[status];
    pub_gnss_pose_->publish(marker);
}

void
Visualizer::on_uwb_received(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // publish uwb pose
    auto marker = create_marker(visualization_msgs::msg::Marker::CUBE);
    marker.header = msg->header;
    marker.pose = msg->pose.pose;

    marker.color = create_color(0, 195, 255);
    pub_uwb_pose_->publish(marker);
}

void
Visualizer::on_odom_received(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // publish odom pose and orientation
    geometry_msgs::msg::PoseStamped ps;
    ps.pose = msg->pose.pose;
    ps.header = msg->header;
    pub_odom_orientation_->publish(ps);
}

visualization_msgs::msg::Marker
Visualizer::create_marker(int32_t marker_type)
{
    visualization_msgs::msg::Marker marker;
    marker.ns = "sgd";
    marker.id = now().nanoseconds();
    marker.type = marker_type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.05;
    marker.lifetime.sec = marker_lifetime_;

    return marker;
}

std_msgs::msg::ColorRGBA
Visualizer::create_color(int r, int g, int b)
{
    std_msgs::msg::ColorRGBA color;
    color.r = (float)r/255;
    color.g = (float)g/255;
    color.b = (float)b/255;
    color.a = 1.0;
    return color;
}

}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_util::Visualizer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
