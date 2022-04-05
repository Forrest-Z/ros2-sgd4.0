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

Visualizer::Visualizer() : rclcpp::Node("visualizer"),
                            in_topics_{"uwb_local", "gps_local"}
{
    RCLCPP_DEBUG(get_logger(), "Creating");
    // Add parameters - a maximum of 6 topics is allowed
    in_topics_ = declare_parameter<std::vector<std::string>>("topics", in_topics_);
    if (in_topics_.size() > 6)
    {
        RCLCPP_ERROR(get_logger(), "Visualizer can display a maximum of 6 topics.");
        return;
    }

    debug_out_dir_ = declare_parameter<std::string>("debug_out_dir", "");   // if dir name is empty no debug file is created
    debug_ = debug_out_dir_.size() > 1;

    // set current time in millis as start time
    time_at_start_ = round(now().nanoseconds() / 1.0E6); // time in millis
    std::string time = std::to_string(time_at_start_);

    if (debug_)
    {
        for (auto str : in_topics_)
        {
            debug_files_.push_back(debug_out_dir_ + "/" + str + "_" + time + ".log");
        }
    }

    // Initialize parameters, pub/sub, services, etc.
    init_pub_sub();
}

Visualizer::~Visualizer()
{
    // Destroy
}

void
Visualizer::init_pub_sub()
{
    publisher = this->create_publisher<visualization_msgs::msg::Marker>("pose_visualization", default_qos);

    for (uint8_t i = 0; i < in_topics_.size(); i++)
    {
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped>)> fnc = std::bind(
            &Visualizer::on_pose_received, this, std::placeholders::_1, (int)i+1);

        RCLCPP_INFO(get_logger(), "Create subscription for topic %s with sensor id %i", in_topics_.at(i).c_str(), i+1);
        subscriber.push_back(this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(in_topics_.at(i), default_qos,
                            fnc));
    }
}

void
Visualizer::on_pose_received(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, int sensor)
{
    auto marker = createMarker();
    marker.header = msg->header;
    marker.pose = msg->pose.pose;
    
    // set color depending on sensor
    std::bitset<3> b{sensor};
    marker.color.a = 1.0;
    marker.color.r = b[0];
    marker.color.g = b[1];
    marker.color.b = b[2];

    publisher->publish(marker);

    if (debug_)
    {
        for (auto str : debug_files_)
        {
            std::fstream out;
            out.open(str, std::ios::out | std::ios::app);
            out << sensor << ";";
            out << time_to_string() << ";";
            out << stamp_to_string(msg->header) << ";";

            out << ";Pose:[";
            out << pose_to_string(msg->pose.pose);
            out << "]\n";
            out.close();
        }
    }
}

visualization_msgs::msg::Marker
Visualizer::createMarker()
{
    visualization_msgs::msg::Marker marker;
    marker.ns = "sgd";
    marker.id = now().nanoseconds();
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.05;
    marker.lifetime.sec = 0;

    return marker;
}

std::string
Visualizer::time_to_string()
{
  double t = now().nanoseconds() / 1.0E9; // - time_at_start_;
  std::string ts = to_string(t);
  return ts;
}

std::string
Visualizer::pose_to_string(geometry_msgs::msg::Pose pose)
{
  std::string s;
  s = std::to_string(pose.position.x) + ";" + std::to_string(pose.position.y) + ";" + std::to_string(pose.position.z);
  return s;
}

std::string
Visualizer::stamp_to_string(std_msgs::msg::Header header)
{
    double t = header.stamp.sec + header.stamp.nanosec / 1.0E9;
    return to_string(t);
}

std::string
Visualizer::to_string(double time_s, std::string format)
{
    int sz = std::snprintf(nullptr, 0, format.c_str(), time_s);
    char buf[sz + 1]; // +1 for null terminator
    std::snprintf(&buf[0], sz+1, format.c_str(), time_s);
    return std::string(buf);
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
