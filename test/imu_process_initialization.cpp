#include "IMU_Processing.hpp"
#include <gtest/gtest.h>

TEST(ImuProcessInitialization, CommitsOnlyStableInputAndHonorsConfiguredGravity) {
  for (const double gravity : {9.79338, 9.80665, 9.809}) {
    ImuProcess process;
    process.setGravity(gravity);
    process.set_acc_cov(V3D::Constant(.1));
    process.set_gyr_cov(V3D::Constant(.1));
    esekfom::esekf<state_ikfom, 12, input_ikfom> filter;
    PointCloudXYZI::Ptr output(new PointCloudXYZI());
    MeasureGroup measurements;
    const V3D acceleration(.2, -.1, .97), bias(.026, -.002, .003);
    auto sample = [&](int i) {
      auto imu = std::make_shared<fast_lio::ImuSample>();
      imu->stamp_ns = int64_t(i) * 5000000;
      imu->linear_acceleration.x() = acceleration.x();
      imu->linear_acceleration.y() = acceleration.y();
      imu->linear_acceleration.z() = acceleration.z();
      imu->angular_velocity.x() = bias.x();
      imu->angular_velocity.y() = bias.y();
      imu->angular_velocity.z() = bias.z();
      measurements.imu.push_back(imu);
    };
    for (int i = 0; i < 20; ++i)
      sample(i);
    measurements.lidar_end_time = .095;
    output->resize(1);
    process.Process(measurements, filter, output);
    EXPECT_TRUE(output->empty());
    EXPECT_DOUBLE_EQ(filter.get_x().bg.norm(), 0.);

    measurements.imu.clear();
    measurements.lidar_beg_time = .1;
    measurements.lidar_end_time = 1.;
    for (int i = 20; i <= 200; ++i)
      sample(i);
    process.Process(measurements, filter, output);
    auto state = filter.get_x();
    EXPECT_TRUE(output->empty());
    EXPECT_LT((state.bg - bias).norm(), 1e-12);
    EXPECT_NEAR(state.grav.vec.norm(), gravity, 1e-12);
    input_ikfom input;
    input.acc = acceleration.normalized() * gravity;
    input.gyro = bias;
    EXPECT_LT(get_f(state, input).segment<3>(12).norm(), 1e-12);
    Eigen::Matrix<double, 23, 1> update = Eigen::Matrix<double, 23, 1>::Zero();
    update[21] = .1;
    state.boxplus(update);
    EXPECT_NEAR(state.grav.vec.norm(), gravity, 1e-12);
  }
}
