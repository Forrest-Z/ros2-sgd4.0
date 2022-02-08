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

#include "sgd_safety/obstacle_checker.hpp"

TEST(TestObstacleChecker, Init)
{
    sgd_safety::ObstacleChecker oc;

    std::vector<float> scan = {2.2,2.4,2.2,5.0,3.0,3.1, 3.2, 4.2, 10.2};

    EXPECT_EQ(oc.compute_speed(scan, -M_PI_2, M_PI_4/2), 0.0)
        << "Obstacle Checker is not initialized, so 0.0 should be returned.";
    EXPECT_THROW(oc.initialize(0.5,1.0,0.98), std::invalid_argument)
        << "Initialization with distance_max < distance_min should fail.";

    EXPECT_TRUE(oc.initialize(0.5,0.5,2.0))
        << "Initialization with valid parameters should return true.";

    // obstacle checker is now initialized so a value greather than 0 is expected
    EXPECT_GT(oc.compute_speed(scan, -M_PI_2, M_PI_4/2), 0.0);
}

TEST(TestObstacleChecker, CalcSpeed)
{
    sgd_safety::ObstacleChecker oc;
    oc.initialize(0.5,0.5,2.0);

    std::vector<float> scan = {2.2,2.4,2.2,5.0,3.0,3.1, 3.2, 4.2, 10.2};
    EXPECT_NEAR(oc.compute_speed(scan, -M_PI_2, M_PI_4/2), 1.0, 1E-6)
        << "All scan values are larger than distance_max, so 1 should be returned.";

    // set a value which should be considered a measurement error
    scan[3] = 0.0;
    EXPECT_NEAR(oc.compute_speed(scan, -M_PI_2, M_PI_4/2), 1.0, 1E-6) 
        << "One value smaller than distance_min should be considered a measurement error";

    // set more than 6 values to 0.3 to detect an obstacle
    for (int i = 1; i < 8; i++)
    {
        scan[i] = 0.2;
    }
    EXPECT_NEAR(oc.compute_speed(scan, -M_PI_4, M_PI_4/4), 0.0, 1E-6)
        << "More than 6 values smaller than distance_min should return a speed of 0.0.";
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}