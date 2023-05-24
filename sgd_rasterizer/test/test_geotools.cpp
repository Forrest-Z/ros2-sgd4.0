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

#include "sgd_util/geotools.hpp"

#define eps 1E-3

TEST(GeotoolsTest, WGStoLocal)
{
    sgd_util::LatLon ll(53.5554544, 10.0218787);

    auto xy1 = sgd_util::LatLon(53.5563072, 10.0218361).to_local(ll);
    EXPECT_NEAR(xy1.first, -2.818, eps);
    EXPECT_NEAR(xy1.second, 94.933, eps);

    auto xy2 = sgd_util::LatLon(53.5536828, 10.023567).to_local(ll);
    EXPECT_NEAR(xy2.first, 111.595, eps);
    EXPECT_NEAR(xy2.second, -197.243, eps);
}

TEST(GeotoolsTest, DistanceLocal)
{
    sgd_util::LatLon ll(53.5554544, 10.0218787);

    // Test various distances; JOSM was taken as reference
    EXPECT_NEAR(ll.distance_to(sgd_util::LatLon(53.5557262, 10.0215752)), 36.308, eps);
    EXPECT_NEAR(ll.distance_to(sgd_util::LatLon(53.5562882, 10.0216364)), 94.191, eps);
    EXPECT_NEAR(ll.distance_to(sgd_util::LatLon(53.5539962, 10.0238513)), 208.246, eps);
    EXPECT_NEAR(ll.distance_to(sgd_util::LatLon(53.5574830, 10.0248699)), 300.201, eps);
}

TEST(GeotoolsTest, DistanceLocalLL)
{
    sgd_util::LatLon ll(53.5554544, 10.0218787);

    // Test various distances; JOSM was taken as reference
    EXPECT_NEAR(ll.distance_to(53.5557262, 10.0215752), 36.308, eps);
    EXPECT_NEAR(ll.distance_to(53.5562882, 10.0216364), 94.191, eps);
    EXPECT_NEAR(ll.distance_to(53.5539962, 10.0238513), 208.246, eps);
    EXPECT_NEAR(ll.distance_to(53.5574830, 10.0248699), 300.201, eps);
}

TEST(GeotoolsTest, InterpolateLL)
{
    sgd_util::LatLon ll(55.0, 10.0);
    sgd_util::LatLon ll2(45.0, 20.0);

    auto vec_ll = ll.interpolate(ll2);
    EXPECT_EQ(vec_ll.size(), 1);

    auto vec_ll2 = ll.interpolate(ll2, 5);
    EXPECT_EQ(vec_ll2.size(), 5);
}

TEST(GeotoolsTest, BearingLL)
{
    sgd_util::LatLon ll(55.0, 10.0);
    sgd_util::LatLon ll2(55.0, 20.0);
    EXPECT_NEAR(ll.bearing(ll2), M_PI_2, eps);

    ll2 = sgd_util::LatLon(45.0, 10.0);
    EXPECT_NEAR(ll.bearing(ll2), M_PI, eps);

    ll2 = sgd_util::LatLon(55.0, 5.0);
    EXPECT_NEAR(ll.bearing(ll2), 1.5*M_PI, eps);

    ll2 = sgd_util::LatLon(65.0, 10.0);
    EXPECT_NEAR(ll.bearing(ll2), 0.0, eps);

    sgd_util::LatLon ll1(53.5551470, 10.0221285);
    sgd_util::LatLon ll3(53.555166, 10.0221542);
    EXPECT_NEAR(ll1.bearing(ll3), 0.6772, eps);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}