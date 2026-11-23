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

#include <cmath>

#include "thesis_perception/motion_estimator.hpp"

TEST(MotionEstimator, StopsPreviewQuicklyAndDoesNotFlickerOnRest)
{
  thesis_perception::MotionEstimator motion;
  double t = 0;
  auto step = [&](double x) {
      t += 0.02;
      motion.update(t, Eigen::Vector3d(x, 0, 0), 0.35, 0.06, 0.025, 2.0);
    };
  for (int i = 0; i < 40; ++i) {step(0.004 * i);}
  ASSERT_TRUE(motion.moving());
  const double stop_x = 0.004 * 39;
  int first_hidden = -1;
  for (int i = 0; i < 25; ++i) {
    step(stop_x + (i % 2 == 0 ? 0.0002 : -0.0002));
    if (!motion.moving() && first_hidden < 0) {first_hidden = i;}
    if (first_hidden >= 0) {EXPECT_FALSE(motion.moving());}
  }
  EXPECT_GE(first_hidden, 0);
  EXPECT_LE(first_hidden, 8);
}

TEST(MotionEstimator, SlowMotionAndReversalAreNotPermanentlySuppressed)
{
  thesis_perception::MotionEstimator motion;
  double t = 0, x = 0;
  for (int i = 0; i < 50; ++i) {
    t += 0.02; x += 0.0014;
    motion.update(t, Eigen::Vector3d(x, 0, 0), 0.35, 0.06, 0.025, 2.0);
  }
  ASSERT_TRUE(motion.moving());
  EXPECT_GT(motion.velocity().x(), 0);
  bool hidden = false;
  for (int i = 0; i < 50; ++i) {
    t += 0.02; x -= 0.0014;
    motion.update(t, Eigen::Vector3d(x, 0, 0), 0.35, 0.06, 0.025, 2.0);
    if (!motion.moving()) {hidden = true;}
  }
  EXPECT_TRUE(hidden);
  EXPECT_TRUE(motion.moving());
  EXPECT_LT(motion.velocity().x(), 0);
}

TEST(MotionEstimator, MaintainsPreviewDuringNoisyContinuousMotion)
{
  thesis_perception::MotionEstimator motion;
  int visible = 0;
  for (int i = 0; i < 150; ++i) {
    const double t = i * 0.02;
    const double x = 0.10 * t + 0.0015 * std::sin(2.0 * 3.141592653589793 * t / 0.20);
    motion.update(t, Eigen::Vector3d(x, 0, 0), 0.35, 0.06, 0.025, 2.0);
    if (i > 25 && motion.moving()) {++visible;}
  }
  EXPECT_GE(visible, 115);
}

TEST(MotionEstimator, PreviewVelocityChangesSmoothlyUnderPositionNoise)
{
  thesis_perception::MotionEstimator motion;
  double last = 0;
  bool have_last = false;
  double max_step = 0;
  int updates = 0;
  for (int i = 0; i < 160; ++i) {
    const double t = i * 0.02;
    const double x = 0.12 * t + 0.002 * std::sin(2.0 * 3.141592653589793 * t / 0.24);
    motion.update(t, Eigen::Vector3d(x, 0, 0), 0.35, 0.06, 0.025, 2.0);
    if (i > 30 && motion.moving()) {
      const double v = motion.velocity().x();
      if (have_last) {max_step = std::max(max_step, std::abs(v - last));}
      last = v;
      have_last = true;
      ++updates;
    }
  }
  EXPECT_GE(updates, 120);
  EXPECT_LT(max_step, 0.006);
}

TEST(MotionEstimator, StablePreviewWithNoisyModerateMovement)
{
  thesis_perception::MotionEstimator motion;
  int visible = 0;
  for (int i = 0; i < 250; ++i) {
    const double t = i * 0.02;
    const double x = 0.12 * t + 0.004 * std::sin(2.0 * 3.141592653589793 * t / 0.2);
    motion.update(t, Eigen::Vector3d(x, 0, 0), 0.35, 0.06, 0.025, 2.0);
    if (i > 30 && motion.moving()) {++visible;}
  }
  EXPECT_GE(visible, 190);
}
