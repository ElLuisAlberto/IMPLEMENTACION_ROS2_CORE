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
#ifndef THESIS_PERCEPTION__STATIONARY_VOXELS_HPP_
#define THESIS_PERCEPTION__STATIONARY_VOXELS_HPP_

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <unordered_map>

namespace thesis_perception
{
struct VoxelKey
{
  int x, y, z;
  bool operator==(const VoxelKey & other) const
  {return x == other.x && y == other.y && z == other.z;}
};
struct VoxelHash
{
  std::size_t operator()(const VoxelKey & k) const
  {
    const auto x = static_cast<std::size_t>(static_cast<unsigned int>(k.x));
    const auto y = static_cast<std::size_t>(static_cast<unsigned int>(k.y));
    const auto z = static_cast<std::size_t>(static_cast<unsigned int>(k.z));
    return (x * 73856093U) ^ (y * 19349663U) ^ (z * 83492791U);
  }
};
class StationaryVoxels
{
  struct Entry
  {
    Eigen::Vector3f center{Eigen::Vector3f::Zero()};
    double first{}, last{};
    unsigned observations{};
    bool stable{};
  };
public:
  StationaryVoxels(float size, float match, double warmup, double retention)
  : size_(size), match_squared_(match * match), warmup_(warmup), retention_(retention) {}
  void clear() {cells_.clear();}
  bool contains(const Eigen::Vector3f & point, double stamp) const
  {
    const auto key = key_for(point);
    for (int x = -1; x <= 1; ++x) {
      for (int y = -1; y <= 1; ++y) {
        for (int z = -1; z <= 1; ++z) {
          const auto found = cells_.find({key.x + x, key.y + y, key.z + z});
          if (found != cells_.end() && found->second.stable &&
            stamp - found->second.last <= retention_ &&
            (found->second.center - point).squaredNorm() <= match_squared_)
          {return true;}
        }
      }
    }
    return false;
  }
  void observe(const Eigen::Vector3f & point, double stamp)
  {
    const auto key = key_for(point);
    auto inserted = cells_.emplace(key, Entry{});
    auto & cell = inserted.first->second;
    if (inserted.second || stamp - cell.last > retention_) {
      cell = Entry{};
      cell.center = point; cell.first = stamp; cell.last = stamp; cell.observations = 1;
      return;
    }
    // At most one observation per frame, independent of local point density.
    if (stamp <= cell.last) {return;}
    if (!cell.stable && stamp - cell.last > 0.2) {
      cell.first = stamp; cell.observations = 0;
    }
    cell.center = 0.85f * cell.center + 0.15f * point;
    cell.last = stamp;
    ++cell.observations;
    if (cell.observations >= 8 && stamp - cell.first >= warmup_) {cell.stable = true;}
  }
  void expire(double stamp)
  {
    for (auto it = cells_.begin(); it != cells_.end();) {
      if (stamp - it->second.last > retention_) {it = cells_.erase(it);} else {++it;}
    }
  }
  std::size_t size() const {return cells_.size();}
private:
  VoxelKey key_for(const Eigen::Vector3f & p) const
  {
    return {static_cast<int>(std::floor(p.x() / size_)),
      static_cast<int>(std::floor(p.y() / size_)),
      static_cast<int>(std::floor(p.z() / size_))};
  }
  float size_, match_squared_;
  double warmup_, retention_;
  std::unordered_map<VoxelKey, Entry, VoxelHash> cells_;
};
}  // namespace thesis_perception
#endif  // THESIS_PERCEPTION__STATIONARY_VOXELS_HPP_
