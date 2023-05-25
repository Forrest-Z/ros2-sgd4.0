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

#ifndef SGD_COSTMAP_PLUGINS__CIRCLE_RASTERIZER_HPP_
#define SGD_COSTMAP_PLUGINS__CIRCLE_RASTERIZER_HPP_

#include <vector>

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

class CircleRasterizer
{
private:
    const int radius_;

    void setOutline(int x, int y)
    {
        // set 8 pixels
        maxs[radius_ + x] = y + radius_;
        maxs[radius_ + y] = x + radius_;
        maxs[radius_ - y] = x + radius_;

        mins[radius_ + x] = y - radius_;
        mins[radius_ + y] = x - radius_;
        mins[radius_ - y] = x - radius_;

        if (x != 0)
        {
            maxs[radius_ - x] = y + radius_;
            mins[radius_ - x] = y - radius_;
        }
    }
public:
    CircleRasterizer(int radius) : radius_(radius < 1 ? 1 : radius)
    {
        PLOGI << "Init CircleRasterizer with radius " << radius_;
        // initialize mins and maxs to size 2*radius 
        mins.resize(2*radius_+1);
        maxs.resize(2*radius_+1);

        // set initial values (origin is at 0,0)
        int x_ = 0;
        int y_ = radius_;
        // save initial pixel
        setOutline(x_, y_);

        // decision parameter
        int dec_param = 3 - 2 * radius_;

        while (y_ >= x_)
        {
            x_++;
            if (dec_param > 0)
            {
                y_--;
                dec_param = dec_param + 4 * (x_ - y_) + 10;
            }
            else
            {
                dec_param = dec_param + 4 * x_ + 6;
            }
            // save pixel
            setOutline(x_, y_);
        }
    }

    ~CircleRasterizer() {}

    std::vector<int> mins;
    std::vector<int> maxs;
};

#endif