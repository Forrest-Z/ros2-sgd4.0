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

#ifndef SGD_UTIL__LOG_UTILS_HPP_
#define SGD_UTIL__LOG_UTILS_HPP_

#include <utility>

#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace sgd_util
{

std::string create_log_dir(std::string sensor_id)
{
    time_t now = time(0);
    tm *ltm = localtime(&now);

    char buf[24];
    std::sprintf(&buf[0], "%4d-%02d-%02d_%02d-%02d-%02d.log", 1900+ltm->tm_year, 1+ltm->tm_mon, ltm->tm_mday,
                            ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
    std::string log_file_(buf);
    return sensor_id + "_" + log_file_;
}

}   // namespace sgd_util
#endif