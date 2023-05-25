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

#ifndef SGD_RASTERIZER__RAY_RASTERIZER_HPP_
#define SGD_RASTERIZER__RAY_RASTERIZER_HPP_

#include <string>
#include <vector>
#include <forward_list>
#include <iostream>

#include <math.h>
#include <algorithm>

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

namespace sgd_rasterizer
{

class RayRasterizer
{
private:
    const int width_, height_;
    float x_lmp, y_bottom, x_rmp, y_top;

    /**
     * @brief Get the intersection object
     * 
     * @param x0 
     * @param y0 
     * @param x1 
     * @param y1 
     * @param py 
     * @return int 
     */
    int get_intersection(float x0, float y0, float x1, float y1, float py);

    /**
     * @brief Fill the data array at the specified y coordinate from 'from' to 'to' with 'what'
     * 
     * @param y
     * @param from first index (inclusive)
     * @param to last index (inclusive)
     * @param what what to insert
     */
    void fill_line(int y, int from, int to, uint8_t what);
public:
    /**
     * @brief Construct a new Ray Rasterizer object
     * 
     * @param width width of the rasterized image
     * @param height height of the rasterized image
     */
    RayRasterizer(int width, int height);
    // RayRasterizer(std::vector<float> vertices, double resolution, int width, int height, std::vector<int8_t> &data);
    ~RayRasterizer();

    /**
     * @brief 
     * 
     * @param vertices list of vertices in local coordinates
     */
    void add_object(std::vector<float> vertices, uint8_t confidence);

    /**
     * @brief Reset ray rasterizer and delete all previously added objects
     * 
     */
    void reset();

    std::vector<uint8_t> data;     /// @brief vector to store the rasterized object
};

} // namespace sgd_rasterizer

#endif
