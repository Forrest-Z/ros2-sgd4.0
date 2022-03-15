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

#include "gtest/gtest.h"

#include "cap_touch/moving_average_filter.hpp"

TEST(MovAvgFilter, Linear)
{
  sgd_hardware_drivers::MovingAverageFilter filter;
  filter.set_filter_size(5);
  
  for (int i = 0; i < 30; i++)
  {
    filter.add_value(i);
    if (i > 5)
    {
      EXPECT_NEAR(filter.mov_avg(), i-2, 1E-6);
    }
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}