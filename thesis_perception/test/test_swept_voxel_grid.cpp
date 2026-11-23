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

#include "thesis_perception/swept_voxel_grid.hpp"

TEST(SweptVoxelGrid, IncludesEndpointsAndIntermediateCellsWithoutDuplicates)
{
  thesis_perception::SweptVoxelGrid grid(0.02, 100, 64);
  grid.add(Eigen::Vector3d(0.005, 0.005, 0.005), Eigen::Vector3d(0.10, 0, 0));
  EXPECT_EQ(grid.cells().size(), 6U);
  for (int x = 0; x <= 5; ++x) {
    EXPECT_EQ(grid.cells().count({x, 0, 0}), 1U);
  }
  grid.add(Eigen::Vector3d(0.005, 0.005, 0.005), Eigen::Vector3d(0.10, 0, 0));
  EXPECT_EQ(grid.cells().size(), 6U);
  EXPECT_FALSE(grid.truncated());
  EXPECT_FALSE(grid.undersampled());
}

TEST(SweptVoxelGrid, ReportsLimitsAndHandlesNegativeCoordinates)
{
  thesis_perception::SweptVoxelGrid limited(0.02, 2, 64);
  limited.add(Eigen::Vector3d(-0.01, 0, 0), Eigen::Vector3d(0.10, 0, 0));
  EXPECT_TRUE(limited.truncated());
  EXPECT_EQ(limited.cells().size(), 2U);
  EXPECT_EQ(limited.cells().count({-1, 0, 0}), 1U);
  thesis_perception::SweptVoxelGrid sparse(0.02, 100, 2);
  sparse.add(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.10, 0, 0));
  EXPECT_TRUE(sparse.undersampled());
}
