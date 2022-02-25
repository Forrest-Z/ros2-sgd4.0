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

TEST(GeotoolsTest, WGStoLocal)
{
  sgd_util::LatLon ll(53.5554544, 10.0218787);

  auto xy1 = sgd_util::LatLon(53.5563072, 10.0218361).to_local(ll);
  EXPECT_NEAR(xy1.first, -2.818, 0.001);
  EXPECT_NEAR(xy1.second, 94.933, 0.001);

  auto xy2 = sgd_util::LatLon(53.5536828, 10.023567).to_local(ll);
  EXPECT_NEAR(xy2.first, 111.595, 0.001);
  EXPECT_NEAR(xy2.second, -197.243, 0.001);

}

TEST(GeotoolsTest, DistanceLocal)
{
  sgd_util::LatLon ll(53.5554544, 10.0218787);

  // Test various distances; JOSM was taken as reference
  EXPECT_NEAR(ll.distance_to(sgd_util::LatLon(53.5557262, 10.0215752)), 36.308, 0.001);
  EXPECT_NEAR(ll.distance_to(sgd_util::LatLon(53.5562882, 10.0216364)), 94.191, 0.001);
  EXPECT_NEAR(ll.distance_to(sgd_util::LatLon(53.5539962, 10.0238513)), 208.246, 0.001);
  EXPECT_NEAR(ll.distance_to(sgd_util::LatLon(53.5574830, 10.0248699)), 300.201, 0.001);
}

TEST(GeotoolsTest, DistanceLocalLL)
{
  sgd_util::LatLon ll(53.5554544, 10.0218787);

  // Test various distances; JOSM was taken as reference
  EXPECT_NEAR(ll.distance_to(53.5557262, 10.0215752), 36.308, 0.001);
  EXPECT_NEAR(ll.distance_to(53.5562882, 10.0216364), 94.191, 0.001);
  EXPECT_NEAR(ll.distance_to(53.5539962, 10.0238513), 208.246, 0.001);
  EXPECT_NEAR(ll.distance_to(53.5574830, 10.0248699), 300.201, 0.001);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}