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

/* OccupancyGrid map input-output library */

#ifndef SGD_MAP_SERVER__MAP_IO_HPP_
#define SGD_MAP_SERVER__MAP_IO_HPP_

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <regex>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sgd_map_server/map_mode.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "tinyxml2.h"
#include "yaml-cpp/yaml.h"

#include <plog/Log.h>
#include <plog/Initializers/RollingFileInitializer.h>

#include "sgd_map_server/Imap_io.hpp"
// #include "sgd_map_server/rasterizer/circle_rasterizer.hpp"
// #include "sgd_map_server/rasterizer/triangle_rasterizer.hpp"
// #include "sgd_map_server/rasterizer/delaunator.hpp"
// #include "sgd_map_server/rasterizer/ray_rasterizer.hpp"

#include "sgd_rasterizer/circle_rasterizer.hpp"
#include "sgd_rasterizer/ray_rasterizer.hpp"

/* Map input part */
namespace sgd_map_server
{

class Map_IO : public IMap_IO
{
private:
    std::regex regex_path_;

    float height_ = 0.0;

    /**
     * @brief Import svg
     * 
     * @param element element to import
     */
    void import_svg(tinyxml2::XMLElement * element, nav_msgs::msg::OccupancyGrid &msg);

    /**
     * @brief Converts a numeric attribute to a float value
     * 
     * @param attribute tinyxml2 Attribute
     * @return float 
     */
    float char2float(const char * attribute);

    bool is_point_inside_polygon(std::vector<double> polygon_outline, float px, float py);

public:
    Map_IO(rclcpp_lifecycle::LifecycleNode::SharedPtr parent);
    ~Map_IO();

    nav_msgs::msg::OccupancyGrid map;

    /**
     * @brief Load the image from map file and generate an OccupancyGrid
     * @param map Output loaded map
     * @throw std::exception
     */
    void loadMapFromFile(const LoadParameters & load_parameters) override;

    /* Map output part */
    /**
     * @brief Checks map saving parameters for consistency
     * @param save_parameters Map saving parameters.
     * NOTE: save_parameters could be updated during function execution.
     * @throw std::exception in case of inconsistent parameters
     */
    void checkSaveParameters(SaveParameters &save_parameters);

    /**
     * @brief Write OccupancyGrid map to file
     * @param map OccupancyGrid map data
     * @return true or false
     */
    bool saveMapToFile(const SaveParameters & save_parameters) override;

    /**
     * @brief Tries to write map data into a file
     * @param map Occupancy grid data
     * @param save_parameters Map saving parameters
     * @throw std::expection in case of problem
     */
    void tryWriteMapToFile(const SaveParameters &save_parameters);
};

} // namespace sgd_map_server

#endif // SGD_MAP_SERVER__MAP_IO_HPP_