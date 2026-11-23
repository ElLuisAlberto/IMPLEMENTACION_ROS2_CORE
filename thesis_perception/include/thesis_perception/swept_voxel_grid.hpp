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
#ifndef THESIS_PERCEPTION__SWEPT_VOXEL_GRID_HPP_
#define THESIS_PERCEPTION__SWEPT_VOXEL_GRID_HPP_

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <unordered_set>

#include "thesis_perception/stationary_voxels.hpp"

namespace thesis_perception
{
// Visualization of sampled trajectories. Not a conservative collision volume.
class SweptVoxelGrid
{
public:
  SweptVoxelGrid(double size, std::size_t max_cells, int max_steps)
  : size_(size), max_cells_(max_cells), max_steps_(max_steps) {}
  void add(const Eigen::Vector3d & start, const Eigen::Vector3d & displacement)
  {
    const double travel = displacement.norm();
    const double required = std::ceil(travel / (size_ * 0.5));
    const int steps = std::max(1, static_cast<int>(std::min(required,
      static_cast<double>(max_steps_))));
    if (required > max_steps_) {undersampled_ = true;}
    for (int i = 0; i <= steps; ++i) {
      const Eigen::Vector3d p = start + displacement * (static_cast<double>(i) / steps);
      const VoxelKey key{static_cast<int>(std::floor(p.x() / size_)),
        static_cast<int>(std::floor(p.y() / size_)), static_cast<int>(std::floor(p.z() / size_))};
      if (cells_.find(key) != cells_.end()) {continue;}
      if (cells_.size() >= max_cells_) {truncated_ = true; return;}
      cells_.insert(key);
    }
  }
  const std::unordered_set<VoxelKey, VoxelHash> & cells() const {return cells_;}
  bool truncated() const {return truncated_;}
  bool undersampled() const {return undersampled_;}
  Eigen::Vector3d center(const VoxelKey & key) const
  {
    return Eigen::Vector3d((key.x + 0.5) * size_, (key.y + 0.5) * size_,
      (key.z + 0.5) * size_);
  }
private:
  double size_;
  std::size_t max_cells_;
  int max_steps_;
  bool truncated_{}, undersampled_{};
  std::unordered_set<VoxelKey, VoxelHash> cells_;
};
}  // namespace thesis_perception
#endif  // THESIS_PERCEPTION__SWEPT_VOXEL_GRID_HPP_
