/* Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* Copyright 2019 Rover Robotics
 * Copyright 2010 Brian Gerkey
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sgd_map_server/map_server.hpp"
#include "sgd_map_server/Imap_io.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace sgd_map_server
{

MapServer::MapServer() : rclcpp_lifecycle::LifecycleNode("map_server")
{
    RCLCPP_INFO(get_logger(), "Creating");

    // initialize plog
    if (!has_parameter("log_dir"))     declare_parameter("log_dir", rclcpp::ParameterValue(".ros/log/"));
    if (!has_parameter("log_severity"))     declare_parameter("log_severity", rclcpp::ParameterValue("I"));

    std::string log_dir, log_sev;
    get_parameter("log_dir", log_dir);
    get_parameter("log_severity", log_sev);
    std::string log_file(log_dir + "/sgd_map_server.log");
    plog::init(plog::severityFromString(log_sev.c_str()), log_file.c_str());
    RCLCPP_INFO(get_logger(), "Save sgd map server log to %s", log_file.c_str());

    // Declare the node parameters
    declare_parameter("yaml_filename");
    declare_parameter("topic_name", "map");
    declare_parameter("topic_vector_map", "map/vector");
    declare_parameter("frame_id", "map");
}

MapServer::~MapServer()
{
    RCLCPP_INFO(get_logger(), "Destroying");
}

CallbackReturn
MapServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "Configuring");

    // Get the name of the YAML file to use
    std::string yaml_filename = get_parameter("yaml_filename").as_string();

    std::string topic_name = get_parameter("topic_name").as_string();
    std::string topic_vector_map = get_parameter("topic_vector_map").as_string();
    frame_id_ = get_parameter("frame_id").as_string();

    // initialize map_io
    map_io = std::make_shared<Map_IO>(shared_from_this());
    map_io_vec = std::make_shared<Map_IO_Vector>(shared_from_this());

    // Shared pointer to LoadMap::Response is also should be initialized
    // in order to avoid null-pointer dereference
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> rsp =
        std::make_shared<nav2_msgs::srv::LoadMap::Response>();

    if (!loadMapResponseFromYaml(yaml_filename, rsp))
    {
        throw std::runtime_error("Failed to load map yaml file: " + yaml_filename);
    }

    // Make name prefix for services
    const std::string service_prefix = get_name() + std::string("/");

    // Create a service that provides the occupancy grid
    occ_service_ = create_service<nav_msgs::srv::GetMap>(
        service_prefix + std::string(service_name_),
        std::bind(&MapServer::getMapCallback, this, _1, _2, _3));

    // Create a publisher using the QoS settings to emulate a ROS1 latched topic
    occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        topic_name,
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    vec_pub_ = create_publisher<sgd_msgs::msg::VecObstacleArray>(
        topic_vector_map, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    // Create a service that loads the occupancy grid from a file
    load_map_service_ = create_service<nav2_msgs::srv::LoadMap>(
        service_prefix + std::string(load_map_service_name_),
        std::bind(&MapServer::loadMapCallback, this, _1, _2, _3));

    return CallbackReturn::SUCCESS;
}

CallbackReturn
MapServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "Activating");

    vec_pub_->on_activate();

    RCLCPP_INFO(get_logger(), "Publish vector map with size %ix%i and %i obstacles.",
        map_io_vec->map.info.width, map_io_vec->map.info.height, map_io_vec->map.obstacles.size());

    auto vec_map = std::make_unique<sgd_msgs::msg::VecObstacleArray>(map_io_vec->map);
    vec_pub_->publish(std::move(vec_map));

    // Publish the map using the latched topic
    occ_pub_->on_activate();
    auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(map_io->map);
    occ_pub_->publish(std::move(occ_grid));

    RCLCPP_INFO(get_logger(), "Activation complete");

    return CallbackReturn::SUCCESS;
}

CallbackReturn
MapServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "Deactivating");

    occ_pub_->on_deactivate();
    vec_pub_->on_deactivate();

    return CallbackReturn::SUCCESS;
}

CallbackReturn
MapServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "Cleaning up");

    occ_pub_.reset();
    vec_pub_.reset();
    occ_service_.reset();
    load_map_service_.reset();

    return CallbackReturn::SUCCESS;
}

CallbackReturn
MapServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "Shutting down");
    return CallbackReturn::SUCCESS;
}

void MapServer::getMapCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<nav_msgs::srv::GetMap::Request> /*request*/,
    std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
{
    // if not in ACTIVE state, ignore request
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        RCLCPP_WARN(
            get_logger(),
            "Received GetMap request but not in ACTIVE state, ignoring!");
        return;
    }
    RCLCPP_INFO(get_logger(), "Handling GetMap request");
    response->map = map_io->map;
}

void MapServer::loadMapCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
    // if not in ACTIVE state, ignore request
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        RCLCPP_WARN(
            get_logger(),
            "Received LoadMap request but not in ACTIVE state, ignoring!");
        return;
    }
    RCLCPP_INFO(get_logger(), "Handling LoadMap request");
    // Load from file
    if (loadMapResponseFromYaml(request->map_url, response))
    {
        auto vec_map = std::make_unique<sgd_msgs::msg::VecObstacleArray>(map_io_vec->map);
        vec_pub_->publish(std::move(vec_map));

        auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(map_io->map);
        occ_pub_->publish(std::move(occ_grid)); // publish new map
    }
}

bool MapServer::loadMapResponseFromYaml(
    const std::string &yaml_file,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
    LOAD_MAP_STATUS load_map_status = LOAD_MAP_SUCCESS;
    if (yaml_file.empty())
    {
        RCLCPP_ERROR(get_logger(), "YAML file name is empty, can't load!");
        response->result = nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST;
        return false;
    }

    RCLCPP_INFO(get_logger(), "Loading yaml file: %s", yaml_file.c_str());
    LoadParameters load_parameters;
    try
    {
        load_parameters = loadMapYaml(yaml_file);
    }
    catch (YAML::Exception &e)
    {
        RCLCPP_ERROR(get_logger(), "Failed processing YAML file %s at position (%d:%d) for reason: %s",
                        yaml_file.c_str(), e.mark.line, e.mark.column, e.what());
        response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_METADATA;
        return false;
    }
    catch (std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "Failed to parse map YAML loaded from file %s for reason: %s", yaml_file.c_str(), e.what());
        response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_METADATA;
        return false;
    }

    try
    {
        map_io->loadMapFromFile(load_parameters);
        map_io_vec->loadMapFromFile(load_parameters);
    }
    catch (std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "Failed to load image file %s for reason: %s", load_parameters.image_file_name.c_str(), e.what());
        response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA;
        return false;
    }

    // load map successful
    // Correcting msg_ header when it belongs to specific node
    updateMsgHeader();

    response->map = map_io->map;
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;

    return true;
}

void MapServer::updateMsgHeader()
{
    msg_.info.map_load_time = now();
    msg_.header.frame_id = frame_id_;
    msg_.header.stamp = now();
}

LoadParameters MapServer::loadMapYaml(const std::string &yaml_filename)
{
    YAML::Node doc = YAML::LoadFile(yaml_filename);
    LoadParameters load_parameters;

    auto image_file_name = yaml_get_value<std::string>(doc, "image");
    if (image_file_name.empty())
    {
        throw YAML::Exception(doc["image"].Mark(), "The image tag was empty.");
    }
    if (image_file_name[0] != '/')
    {
        // dirname takes a mutable char *, so we copy into a vector
        std::vector<char> fname_copy(yaml_filename.begin(), yaml_filename.end());
        fname_copy.push_back('\0');
        image_file_name = std::string(dirname(fname_copy.data())) + '/' + image_file_name;
    }
    load_parameters.image_file_name = image_file_name;

    load_parameters.resolution = yaml_get_value<double>(doc, "resolution");
    load_parameters.width = yaml_get_value<uint>(doc, "width");
    load_parameters.height = yaml_get_value<uint>(doc, "height");
    load_parameters.origin = yaml_get_value<std::vector<double>>(doc, "origin");
    if (load_parameters.origin.size() != 3)
    {
        throw YAML::Exception(
            doc["origin"].Mark(), "value of the 'origin' tag should have 3 elements, not " +
                                        std::to_string(load_parameters.origin.size()));
    }

    load_parameters.free_thresh = yaml_get_value<double>(doc, "free_thresh");
    load_parameters.occupied_thresh = yaml_get_value<double>(doc, "occupied_thresh");

    auto map_mode_node = doc["mode"];
    if (!map_mode_node.IsDefined())
    {
        load_parameters.mode = MapMode::Trinary;
    }
    else
    {
        load_parameters.mode = map_mode_from_string(map_mode_node.as<std::string>());
    }

    try
    {
        load_parameters.negate = yaml_get_value<int>(doc, "negate");
    }
    catch (YAML::Exception &)
    {
        load_parameters.negate = yaml_get_value<bool>(doc, "negate");
    }

    RCLCPP_INFO(get_logger(), "resolution: %.2f", load_parameters.resolution);
    RCLCPP_INFO(get_logger(), "origin[0]: %.2f", load_parameters.origin[0]);
    RCLCPP_INFO(get_logger(), "origin[1]: %.2f", load_parameters.origin[1]);
    RCLCPP_INFO(get_logger(), "origin[2]: %.2f", load_parameters.origin[2]);
    RCLCPP_INFO(get_logger(), "free_thresh: %.2f", load_parameters.free_thresh);
    RCLCPP_INFO(get_logger(), "occupied_thresh: %.2f", load_parameters.occupied_thresh);
    RCLCPP_INFO(get_logger(), "mode: %s", map_mode_to_string(load_parameters.mode));
    RCLCPP_INFO(get_logger(), "negate: ", load_parameters.negate);

    return load_parameters;
}

} // namespace nav2_map_server

int main(int argc, char **argv)
{
    std::string node_name("map_server");

    rclcpp::init(argc, argv);
    auto node = std::make_shared<sgd_map_server::MapServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
