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

#include <string>
#include "gtest/gtest.h"

#include "sgd_global_planner/a_star.hpp"

static std::string BASE_PATH("");
static std::string ROS_LOG_DIR("");

TEST(TestAStar, ComputeWayDest)
{
  // compute way for different destinations
  std::shared_ptr<nav_sgd::A_Star_Users> a_star_users = std::make_shared<nav_sgd::A_Star_Users>(BASE_PATH + "users.xml");
  auto t = std::chrono::system_clock::now().time_since_epoch().count();
  nav_sgd::A_Star a_star(BASE_PATH + "lohmuehlenpark.nav", a_star_users, ROS_LOG_DIR + "a_star_" + std::to_string(t) + ".log");
}

TEST(TestAStar, ComputeWayUsers)
{
  // compute way for different users
}

TEST(TestAStar, ComputeWayLength)
{
  // compute way and check length
}

int main(int argc, char **argv)
{
  auto exe = std::string(argv[0]);
  BASE_PATH = exe.substr(0,exe.find_last_of("/")) + "/test/data/";
  ROS_LOG_DIR = exe.substr(0, exe.find_last_of("/")) + "../log/";

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}