#include "imu_initialization.h"
#include <gtest/gtest.h>
#include <limits>

using fast_lio::ImuInitializationWindow;
using Eigen::Vector3d;

TEST(ImuInitialization, RequiresTimeAndSamplesBeforeAcceptingStaticInput) {
  ImuInitializationWindow window;
  for (int i = 0; i < 20; ++i)
    window.add(i * .005, Vector3d(0, 0, 1), Vector3d::Zero());
  EXPECT_FALSE(window.estimate().ready);
  for (int i = 20; i <= 200; ++i)
    window.add(i * .005, Vector3d(0, 0, 1), Vector3d::Zero());
  EXPECT_TRUE(window.estimate().ready);
}

TEST(ImuInitialization, DiscardsStartupMotionBeforeEstimatingGravityAndBias) {
  ImuInitializationWindow window;
  for (int i = 0; i <= 200; ++i)
    window.add(i * .005, Vector3d(.4, 0, 1), Vector3d(0, 0, .3));
  EXPECT_FALSE(window.estimate().ready);
  const Vector3d acceleration(.2, -.1, .97), bias(.001, -.002, .003);
  for (int i = 201; i < 300; ++i)
    window.add(i * .005, acceleration, bias);
  EXPECT_FALSE(window.estimate().ready);
  for (int i = 300; i <= 402; ++i)
    window.add(i * .005, acceleration, bias);
  const auto result = window.estimate();
  ASSERT_TRUE(result.ready);
  EXPECT_LT((result.acceleration - acceleration).norm(), 1e-12);
  EXPECT_LT((result.angular_rate - bias).norm(), 1e-12);
}

TEST(ImuInitialization, DoesNotForceAcceptPersistentMotion) {
  ImuInitializationWindow rotation, acceleration;
  for (int i = 0; i <= 2000; ++i) {
    rotation.add(i * .005, Vector3d(0, 0, 1), Vector3d(0, 0, .3));
    acceleration.add(i * .005, Vector3d(.2 * std::sin(i), 0, 1), Vector3d::Zero());
  }
  EXPECT_FALSE(rotation.estimate().ready);
  EXPECT_FALSE(acceleration.estimate().ready);
}

TEST(ImuInitialization, AcceptsStableNoisyInputWithGyroBiasButRejectsChangingMeans) {
  ImuInitializationWindow stationary, settling;
  for (int i = 0; i <= 200; ++i) {
    const Vector3d noise(.022 * (i % 2 ? 1 : -1), 0, 0);
    stationary.add(i * .005, Vector3d(0, 0, 1) + noise, Vector3d(.026, 0, 0));
    settling.add(i * .005, Vector3d(.02 * i / 200., 0, 1), Vector3d::Zero());
  }
  EXPECT_TRUE(stationary.estimate().ready);
  EXPECT_FALSE(settling.estimate().ready);
}

TEST(ImuInitialization, DecisionsAndDirectionVarianceAreAccelerationUnitInvariant) {
  ImuInitializationWindow g, si;
  for (int i = 0; i <= 200; ++i) {
    const Vector3d a(.2 + .002 * std::sin(i), -.1, .97);
    g.add(i * .005, a, Vector3d::Zero());
    si.add(i * .005, 9.80665 * a, Vector3d::Zero());
  }
  const auto first = g.estimate(), second = si.estimate();
  ASSERT_TRUE(first.ready);
  ASSERT_TRUE(second.ready);
  EXPECT_NEAR(first.acceleration_std_g, second.acceleration_std_g, 1e-14);
  EXPECT_NEAR(first.gravityDirectionVariance(), second.gravityDirectionVariance(), 1e-16);
  EXPECT_LT((first.acceleration.normalized() - second.acceleration.normalized()).norm(), 1e-14);
}

TEST(ImuInitialization, ResetsOnInvalidInputClockRollbackAndDataGap) {
  ImuInitializationWindow window;
  for (int i = 0; i <= 200; ++i)
    window.add(i * .005, Vector3d(0, 0, 1), Vector3d::Zero());
  ASSERT_TRUE(window.estimate().ready);
  const auto count = window.estimate().samples;
  window.add(1., Vector3d(0, 0, 1), Vector3d::Zero());
  EXPECT_EQ(window.estimate().samples, count);
  window.add(2., Vector3d(0, 0, 1), Vector3d::Zero());
  EXPECT_FALSE(window.estimate().ready);
  EXPECT_EQ(window.estimate().samples, 1u);
  window.add(.1, Vector3d(0, 0, 1), Vector3d::Zero());
  EXPECT_EQ(window.estimate().samples, 1u);
  window.add(.105, Vector3d(std::numeric_limits<double>::quiet_NaN(), 0, 1), Vector3d::Zero());
  EXPECT_EQ(window.estimate().samples, 0u);
  window.add(.110, Vector3d::Zero(), Vector3d::Zero());
  EXPECT_EQ(window.estimate().samples, 0u);
}

TEST(ImuInitialization, BoundedStorageDoesNotBypassTimeRequirement) {
  ImuInitializationWindow window;
  for (int i = 0; i < 10000; ++i)
    window.add(i * .00001, Vector3d(0, 0, 1), Vector3d::Zero());
  EXPECT_LE(window.estimate().samples, 2048u);
  EXPECT_FALSE(window.estimate().ready);
}
