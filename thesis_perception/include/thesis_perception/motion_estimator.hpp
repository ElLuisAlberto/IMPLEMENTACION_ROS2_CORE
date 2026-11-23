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
#ifndef THESIS_PERCEPTION__MOTION_ESTIMATOR_HPP_
#define THESIS_PERCEPTION__MOTION_ESTIMATOR_HPP_

#include <Eigen/Core>
#include <algorithm>
#include <deque>
#include <utility>

namespace thesis_perception
{
class MotionEstimator
{
public:
  void update(double stamp, const Eigen::Vector3d & center, double window,
    double enter_speed, double exit_speed, double max_speed)
  {
    samples_.emplace_back(stamp, center);
    while (samples_.size() > 2 && stamp - samples_.front().first > window) {
      samples_.pop_front();
    }
    Eigen::Vector3d long_velocity, recent_velocity;
    if (!slope(window, 0.12, long_velocity) || !slope(0.16, 0.06, recent_velocity)) {
      moving_ = false;
      velocity_valid_ = false;
      velocity_.setZero();
      enter_ = exit_ = 0;
      return;
    }
    const double long_speed = long_velocity.norm();
    const double recent_speed = recent_velocity.norm();
    const double dot = long_velocity.dot(recent_velocity);
    const bool aligned = dot > 0.6 * long_speed * recent_speed;
    if (!moving_) {
      enter_ = long_speed >= enter_speed && recent_speed >= enter_speed && aligned ? enter_ + 1 : 0;
      if (enter_ >= 3) {moving_ = true; exit_ = 0;}
    } else {
      // A fast fall or reversal in recent motion makes the long-window preview stale.
      const bool stopped = recent_speed <= exit_speed;
      const bool braking = long_speed >= enter_speed && recent_speed < 0.55 * long_speed;
      const bool reversing = dot < 0;
      exit_ = stopped || braking || reversing ? exit_ + 1 : 0;
      const int required = (stopped || reversing) ? 2 : 4;
      if (exit_ >= required) {moving_ = false; enter_ = 0; exit_ = 0;}
    }
    Eigen::Vector3d bounded = long_velocity;
    if (long_speed > max_speed) {bounded *= max_speed / long_speed;}
    if (moving_) {
      if (velocity_valid_) {
        velocity_ = 0.75 * velocity_ + 0.25 * bounded;
      } else {
        velocity_ = bounded;
        velocity_valid_ = true;
      }
    } else {
      velocity_.setZero();
      velocity_valid_ = false;
    }
  }
  bool moving() const {return moving_;}
  const Eigen::Vector3d & velocity() const {return velocity_;}

private:
  bool slope(double interval, double min_span, Eigen::Vector3d & result) const
  {
    if (samples_.size() < 4) {return false;}
    const double last = samples_.back().first;
    auto begin = samples_.begin();
    while (begin != samples_.end() && last - begin->first > interval) {++begin;}
    const auto count = std::distance(begin, samples_.end());
    if (count < 3 || last - begin->first < min_span) {return false;}
    double mean_t = 0;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (auto it = begin; it != samples_.end(); ++it) {
      mean_t += it->first - last;
      mean += it->second;
    }
    mean_t /= static_cast<double>(count);
    mean /= static_cast<double>(count);
    double variance = 0;
    Eigen::Vector3d covariance = Eigen::Vector3d::Zero();
    for (auto it = begin; it != samples_.end(); ++it) {
      const double delta = it->first - last - mean_t;
      variance += delta * delta;
      covariance += delta * (it->second - mean);
    }
    if (variance <= 1e-9) {return false;}
    result = covariance / variance;
    return true;
  }
  std::deque<std::pair<double, Eigen::Vector3d>> samples_;
  Eigen::Vector3d velocity_{Eigen::Vector3d::Zero()};
  int enter_{}, exit_{};
  bool moving_{}, velocity_valid_{};
};
}  // namespace thesis_perception
#endif  // THESIS_PERCEPTION__MOTION_ESTIMATOR_HPP_
