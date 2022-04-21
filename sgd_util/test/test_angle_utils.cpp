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

#include "sgd_util/angle_utils.hpp"

#define eps 1E-3

TEST(AngleUtilsTest, SimpleAngle)
{
    // angle returns an angle in [-PI, PI]
    double x1 = 3.0;
    double y1 = 1.0;

    EXPECT_NEAR(sgd_util::angle(x1, y1, 3.0, 5.0), 1.5708, eps);
    EXPECT_NEAR(sgd_util::angle(x1, y1, 5.0, 1.0), 0.0, eps);
    EXPECT_NEAR(sgd_util::angle(x1, y1, 0.0, -2.0), -2.3562, eps);
    EXPECT_NEAR(sgd_util::angle(x1, y1, 6.0, -2.0), -0.7854, eps);
}

TEST(AngleUtilsTest, DeltaAngle)
{
    double x2 = 3.0;
    double y2 = 1.0;

    // right angle
    EXPECT_NEAR(sgd_util::delta_angle(-1.0, 1.0, x2, y2, 3.0, 5.0), 1.5708, eps);
    EXPECT_NEAR(sgd_util::delta_angle(6.0, 1.0, x2, y2, 3.0, 5.0), 1.5708, eps);
    EXPECT_NEAR(sgd_util::delta_angle(6.0, -2.0, x2, y2, 0.0, -2.0), 1.5708, eps);
    EXPECT_NEAR(sgd_util::delta_angle(0.0, 4.0, x2, y2, 0.0, -2.0), 1.5708, eps);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}