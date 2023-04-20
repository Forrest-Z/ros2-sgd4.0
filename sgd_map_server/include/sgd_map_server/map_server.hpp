// Copyright (c) 2018 Intel Corporation
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

#ifndef SGD_MAP_SERVER__MAP_SERVER_HPP_
#define SGD_MAP_SERVER__MAP_SERVER_HPP_

#include <string>
#include <memory>
#include <functional>
#include <fstream>
#include <stdexcept>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
// #include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav2_msgs/srv/load_map.hpp"

#include "yaml-cpp/yaml.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "sgd_map_server/map_io.hpp"
#include "sgd_map_server/map_io_vector.hpp"
#include "sgd_msgs/msg/vec_obstacle_array.hpp"

namespace sgd_map_server
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @class sgd_map_server::MapServer
 * @brief Parses the map yaml file and creates a service and a publisher that
 * provides occupancy grid
 */
class MapServer : public rclcpp_lifecycle::LifecycleNode
{

    typedef enum
    {
        LOAD_MAP_SUCCESS,
        MAP_DOES_NOT_EXIST,
        INVALID_MAP_METADATA,
        INVALID_MAP_DATA
    } LOAD_MAP_STATUS;

public:
    /**
     * @brief A constructor for sgd_map_server::MapServer
     */
    MapServer();

    /**
     * @brief A Destructor for sgd_map_server::MapServer
     */
    ~MapServer();

protected:
    /**
     * @brief Sets up required params and services. Loads map and its parameters from the file
     * @param state Lifecycle Node's state
     * @return Success or Failure
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    /**
     * @brief Start publishing the map using the latched topic
     * @param state Lifecycle Node's state
     * @return Success or Failure
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    /**
     * @brief Stops publishing the latched topic
     * @param state Lifecycle Node's state
     * @return Success or Failure
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    /**
     * @brief Resets the member variables
     * @param state Lifecycle Node's state
     * @return Success or Failure
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    /**
     * @brief Called when in Shutdown state
     * @param state Lifecycle Node's state
     * @return Success or Failure
     */
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

    /**
     * @brief Load the map YAML, image from map file name and
     * generate output response containing an OccupancyGrid.
     * Update msg_ class variable.
     * @param yaml_file name of input YAML file
     * @param response Output response with loaded OccupancyGrid map
     * @return true or false
     */
    bool loadMapResponseFromYaml(
        const std::string &yaml_file,
        std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response);

    /**
     * @brief Load and parse the given YAML file
     * @param yaml_filename Name of the map file passed though parameter
     * @return Map loading parameters obtained from YAML file
     * @throw YAML::Exception
     */
    LoadParameters loadMapYaml(const std::string & yaml_filename);

    /**
     * @brief Method correcting msg_ header when it belongs to instantiated object
     */
    void updateMsgHeader();

    /**
     * @brief Map getting service callback
     * @param request_header Service request header
     * @param request Service request
     * @param response Service response
     */
    void getMapCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
        std::shared_ptr<nav_msgs::srv::GetMap::Response> response);

    /**
     * @brief Map loading service callback
     * @param request_header Service request header
     * @param request Service request
     * @param response Service response
     */
    void loadMapCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
        std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response);

    // The name of the service for getting a map
    const std::string service_name_{"map"};

    // The name of the service for loading a map
    const std::string load_map_service_name_{"load_map"};

    // A service to provide the occupancy grid (GetMap) and the message to return
    rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr occ_service_;

    // A service to load the occupancy grid from file at run time (LoadMap)
    rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr load_map_service_;

    // A topic on which the occupancy grid will be published
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;
    // A topic on which the vector map will be published
    rclcpp_lifecycle::LifecyclePublisher<sgd_msgs::msg::VecObstacleArray>::SharedPtr vec_pub_;

    // The frame ID used in the returned OccupancyGrid message
    std::string frame_id_;

    // The message to publish on the occupancy grid topic
    nav_msgs::msg::OccupancyGrid msg_;

    // Map_IO class
    std::shared_ptr<Map_IO> map_io;
    // load vectorized image
    std::shared_ptr<Map_IO_Vector> map_io_vec;

private:
    /**
   * @brief Get the given subnode value. The only reason this function exists is to wrap 
   * the exceptions in slightly nicer error messages, including the name of the failed key
   * 
   * @tparam T 
   * @param node 
   * @param key 
   * @return T 
   * @throw YAML::Exception
   */
  template<typename T>
  T yaml_get_value(const YAML::Node & node, const std::string & key)
  {
    try {
      return node[key].as<T>();
    } catch (YAML::Exception & e) {
      std::stringstream ss;
      ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
      throw YAML::Exception(e.mark, ss.str());
    }
  }

};

} // namespace sgd_map_server

#endif // SGD_MAP_SERVER__MAP_SERVER_HPP_
