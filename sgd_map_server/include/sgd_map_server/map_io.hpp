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

//#include "sgd_util/log_utils.hpp"
#include "sgd_map_server/map_mode.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "tinyxml2.h"
#include "yaml-cpp/yaml.h"

#include <plog/Log.h>
#include <plog/Initializers/RollingFileInitializer.h>

/* Map input part */
namespace sgd_map_server
{

enum LOAD_MAP_STATUS
{
    LOAD_MAP_SUCCESS,
    MAP_DOES_NOT_EXIST,
    INVALID_MAP_METADATA,
    INVALID_MAP_DATA
};

enum OBSTACLE_TYPE
{
    UNDEFINED,
    NATURAL,
    BARRIER,
    WALL
};

struct xy
{
    float x,y;
};

struct Polygon
{
    uint16_t id;    // unique id
    std::vector<xy> points;     // vertices
    OBSTACLE_TYPE type;
    bool is_closed;             // set to true if first point equals last point
};

struct Circle
{
    uint16_t id;    // unique id
    xy pos;         // position
    OBSTACLE_TYPE type;
    float r;        // radius
};

struct LoadParameters
{
    std::string image_file_name;
    double resolution{0};
    std::vector<double> origin{0, 0, 0};
    double free_thresh;
    double occupied_thresh;
    MapMode mode;
    bool negate;
};

struct SaveParameters
{
    std::string map_file_name{""};
    std::string image_format{""};
    double free_thresh{0.0};
    double occupied_thresh{0.0};
    MapMode mode{MapMode::Trinary};
};

class Map_IO
{
private:
    std::vector<Polygon> polygons;
    std::vector<Circle> circles;

    std::regex regex_fill_;
    std::regex regex_path_;

    /**
     * @brief Import svg
     * 
     * @param element element to import
     */
    void import_svg(tinyxml2::XMLElement * element);

    /**
     * @brief Converts a numeric attribute to a float value
     * 
     * @param attribute tinyxml2 Attribute
     * @return float 
     */
    float char2float(const char * attribute);

    /**
     * @brief 
     * 
     * @param style svg style attribute as string
     * @return OBSTACLE_TYPE 
     */
    OBSTACLE_TYPE parse_style_attribute(std::string style);

public:
    Map_IO(std::string log_dir, std::string log_sev);
    ~Map_IO();

    /**
     * @brief Load the image from map file and generate an OccupancyGrid
     * @param load_parameters Parameters of loading map
     * @param map Output loaded map
     * @throw std::exception
     */
    void loadMapFromFile(
        const LoadParameters &load_parameters,
        nav_msgs::msg::OccupancyGrid &map);

    /* Map output part */
    /**
     * @brief Checks map saving parameters for consistency
     * @param save_parameters Map saving parameters.
     * NOTE: save_parameters could be updated during function execution.
     * @throw std::exception in case of inconsistent parameters
     */
    void checkSaveParameters(
        SaveParameters &save_parameters);

    /**
     * @brief Write OccupancyGrid map to file
     * @param map OccupancyGrid map data
     * @param save_parameters Map saving parameters.
     * @return true or false
     */
    bool saveMapToFile(
        const nav_msgs::msg::OccupancyGrid &map,
        const SaveParameters &save_parameters);

    /**
     * @brief Tries to write map data into a file
     * @param map Occupancy grid data
     * @param save_parameters Map saving parameters
     * @throw std::expection in case of problem
     */
    void tryWriteMapToFile(
        const nav_msgs::msg::OccupancyGrid &map,
        const SaveParameters &save_parameters);
};

} // namespace sgd_map_server

#endif // SGD_MAP_SERVER__MAP_IO_HPP_