// Copyright 2026 Luis Alberto Munoz Marin
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
#include <gtest/gtest.h>

#include "thesis_perception/stationary_voxels.hpp"

TEST(StationaryVoxels, RequiresHistoryRetainsOcclusionAndExpires)
{
  thesis_perception::StationaryVoxels map(0.02f, 0.02f, 1.0, 3.0);
  const Eigen::Vector3f background(0.04f, 0.04f, 0.8f);
  for (int i = 0; i < 15; ++i) {map.observe(background, i * 0.05);}
  EXPECT_FALSE(map.contains(background, 0.7));
  for (int i = 15; i <= 24; ++i) {map.observe(background, i * 0.05);}
  EXPECT_TRUE(map.contains(background, 1.2));
  EXPECT_TRUE(map.contains(background, 3.0));
  EXPECT_FALSE(map.contains(background, 4.3));
  const Eigen::Vector3f foreground(0.04f, 0.04f, 0.76f);
  EXPECT_FALSE(map.contains(foreground, 1.2));
  map.expire(4.3);
  EXPECT_EQ(map.size(), 0U);
}

TEST(StationaryVoxels, SparseHistoryAndResetDoNotConfirmBackground)
{
  thesis_perception::StationaryVoxels map(0.02f, 0.02f, 1.0, 3.0);
  const Eigen::Vector3f point(0.04f, 0.04f, 0.8f);
  for (int i = 0; i < 10; ++i) {map.observe(point, i * 0.3);}
  EXPECT_FALSE(map.contains(point, 2.7));
  map.clear();
  EXPECT_EQ(map.size(), 0U);
}
