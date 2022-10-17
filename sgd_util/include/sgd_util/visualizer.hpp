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

#ifndef SGD_UTIL__VISUALIZER_HPP_
#define SGD_UTIL__VISUALIZER_HPP_

#include <chrono>
#include <iostream>
#include <fstream>
#include <bitset>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sgd_util/geotools.hpp"

#include "tinyxml2.h"

namespace sgd_util
{

class Visualizer : public rclcpp::Node
{

public:
  Visualizer();
  ~Visualizer();

protected:
  //! \brief Init parameters
  bool debug_;  // internal variable to check if debugging is enabled
  std::string debug_out_dir_;
  std::vector<std::string> debug_files_;
  std::vector<std::string> in_topics_;

  //! \brief Init publisher and subscriber
  void init_pub_sub();
  rclcpp::QoS default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_map_marker;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> subscriber;

  double time_at_start_;

  bool is_map_published_;
  void pub_map_markers();

  void on_pose_received(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, uint8_t sensor);
  visualization_msgs::msg::Marker createMarker();

  std::string time_to_string();
  std::string stamp_to_string(std_msgs::msg::Header header);
  std::string to_string(double time_s, std::string format="%.3f");
  std::string pose_to_string(geometry_msgs::msg::Pose pose);
};

}   // namespace sgd_util

#endif  // SGD_UTIL__VISUALIZER_HPP_