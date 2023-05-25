// Copyright 2023 HAW Hamburg
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

#ifndef SGD_COSTMAP_PLUGINS__TRIANGLE_RASTERIZER_HPP_
#define SGD_COSTMAP_PLUGINS__TRIANGLE_RASTERIZER_HPP_

#include <string>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <chrono>

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

namespace sgd_costmap_plugins
{

class TriangleRasterizer
{
private:
    // lmp = leftmost point, rmp = rightmost point
    int x_lmp_;     /// leftmost point in x direction
    int y_lmp_;     /// leftmost point in y direction
    int x1_;        /// x in the middle
    int y1_;        /// y in the middle
    int x_rmp_;     /// rightmost point in x direction
    int y_rmp_;     /// rightmost point in y direction

    /**
     * @brief Adds a point to the outline
     * 
     * @param x x value of the point
     * @param y y value of the point
     */
    void setOutlinePnt(int x, int y);

    /**
     * @brief Use bresenhams algorithm to calculate line from point (x1, y1) to point (x2, y2)
     * 
     * @param x1 x value of point 1
     * @param y1 y value of point 1
     * @param x2 x value of point 2
     * @param y2 y value of point 2
     */
    void plotPixel(int x1, int y1, int x2, int y2);
public:
    /**
     * @brief Initialize TriangleRasterizer and calculates the outline
     * of the triangle from the given points.
     * For more information on the algorithm see the provided Readme.md.
     * 
     * @param x0 x value of first triangle point
     * @param y0 y value of first triangle point
     * @param x1 x value of second triangle point
     * @param y1 y value of second triangle point
     * @param x2 x value of third triangle point
     * @param y2 y value of third triangle point
     */
    TriangleRasterizer(int x0, int y0, int x1, int y1, int x2, int y2);
    ~TriangleRasterizer();

    std::vector<int> mins;      /// the lower edge of the outline
    std::vector<int> maxs;      /// the upper edge of the outline

    /**
     * @brief Get the x coordinate of the leftmost point in global coordinates * resolution
     * 
     * @return int 
     */
    inline int get_lmp()
    {
        return x_lmp_;
    }
};


} // namespace sgd_costmap_plugins

#endif