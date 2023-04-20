// Copyright 2021 HAW Hamburg
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

#ifndef SGD_MAP_SERVER__IMAP_IO_HPP_
#define SGD_MAP_SERVER__IMAP_IO_HPP_

#pragma once

#include <string>
#include <variant>
#include <unordered_map>
#include <vector>
#include <variant>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sgd_map_server/map_mode.hpp"

namespace sgd_map_server
{

struct LoadParameters
{
    std::string image_file_name;
    double resolution{0};
    uint width;
    uint height;
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

class IMap_IO
{
public:
    virtual ~IMap_IO() = default;

    /**
     * @brief Load the image from map file and generate an OccupancyGrid
     * @param map Output loaded map
     * @throw std::exception
     */
    virtual void loadMapFromFile(const LoadParameters & load_parameters) = 0;

    /**
     * @brief Write OccupancyGrid map to file
     * @param map OccupancyGrid map data
     * @return true or false
     */
    virtual bool saveMapToFile(const SaveParameters & save_parameters) = 0;
    
};

} // namespace sgd_map_server

#endif // SGD_MAP_SERVER__IMAP_IO_HPP_