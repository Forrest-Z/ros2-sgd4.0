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

#ifndef SGD_LOCALIZATION_OBSTACLE_HPP
#define SGD_LOCALIZATION_OBSTACLE_HPP

#include <stdint.h>
#include <iostream>
#include <vector>
#include <list>
#include <math.h>

#include "geometry_msgs/msg/pose.hpp"

#include "point.hpp"
#include "sgd_msgs/msg/vec_obstacle.hpp"

namespace sgd_localization
{

class Obstacle
{
enum type {polygon, circle};

private:
    uint32_t id_;           // unique id
    type type_;
    bool is_closed_;        // true if polygon is closed
    
    /**
     * @brief Signum function with sgn(0) = 1
     * 
     * @return int8_t 1 if number >= 0 otherwise -1
     */
    inline int8_t sgn(float number)
    {
        return number >= 0.0 ? 1 : -1;
    }
    
public:

    Obstacle();
    ~Obstacle();

    std::vector<Point> vertices_;      // vector containing all vertices

    /**
     * @brief Compute the expected scan for this obstacle from the current viewpoint
     * 
     * @param pose 
     * @return std::vector<Point> 
     */
    std::vector<Point> expected_scan(float robo_x, float robo_y, float robo_yaw);

    /**
     * @brief Provides access to points contained in the vertices vector. The function
     * allows to define index values larger than the vector size without producing an overflow error.
     * The function behaves like accessing a circular vector.
     * 
     * @param index 
     * @return Point at position index%vertices.size()
     */
    Point get_vertex(int index)
    {
        return vertices_[(int)(index%vertices_.size())];
    }

    void add_vertex(float x, float y);
    sgd_msgs::msg::VecObstacle to_msg();
};

}       // namespace sgd_local_costmap

#endif