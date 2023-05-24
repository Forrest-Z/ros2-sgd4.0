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

#ifndef SGD_RASTERIZER__CIRCLE_RASTERIZER_HPP_
#define SGD_RASTERIZER__CIRCLE_RASTERIZER_HPP_

#include <vector>

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"

namespace sgd_rasterizer
{

class CircleRasterizer
{
private:
    const int radius_;

    void setOutline(int x, int y);
public:
    CircleRasterizer(int radius);
    ~CircleRasterizer() {}

    std::vector<int> mins;
    std::vector<int> maxs;
};

} // namespace sgd_map_server

#endif