#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>

namespace fast_lio {

struct ImuInitializationEstimate {
  bool ready = false;
  size_t samples = 0;
  double duration_s = 0.;
  double acceleration_std_g = 0.;
  double angular_rate_rms = 0.;
  double acceleration_mean_change_g = 0.;
  double angular_rate_mean_change = 0.;
  double acceleration_mean_variance = 0.;
  double angular_rate_mean_variance = 0.;
  double acceleration_mean_std_g = 0.;
  double angular_rate_mean_std = 0.;
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_rate = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration_variance = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_rate_variance = Eigen::Vector3d::Zero();

  double gravityDirectionVariance() const {
    // Acceleration and its variance must use the same units, including g inputs.
    return acceleration_mean_variance / (3. * acceleration.squaredNorm());
  }
};

// Static startup needs a recent, continuous window. Older motion must not bias
// the gravity/bias average after the sensor settles. No forced timeout accept.
class ImuInitializationWindow {
  struct Sample {
    double time;
    Eigen::Vector3d acceleration, angular_rate;
  };
  std::deque<Sample, Eigen::aligned_allocator<Sample>> samples_;

public:
  static constexpr double kDurationS = 1.;
  static constexpr double kMaxGapS = .1;
  // Noise and a constant gyro bias are allowed; changing means are not.
  static constexpr double kMaxAngularRateRms = .05;
  static constexpr double kMaxAccelerationMeanChangeG = .005;
  static constexpr double kMaxAngularRateMeanChange = .01;
  // Mean-estimation error uses half the existing between-half stability budget.
  static constexpr double kMaxAccelerationMeanStdG = kMaxAccelerationMeanChangeG / 2.;
  static constexpr double kMaxAngularRateMeanStd = kMaxAngularRateMeanChange / 2.;
  static constexpr size_t kMinSamples = 20;
  static constexpr size_t kMaxSamples = 2048;

  void reset() { samples_.clear(); }

  void add(double time, const Eigen::Vector3d& acceleration,
           const Eigen::Vector3d& angular_rate) {
    if (!std::isfinite(time) || !acceleration.allFinite() ||
        !angular_rate.allFinite() || acceleration.norm() < 1e-6) {
      reset();
      return;
    }
    if (!samples_.empty()) {
      if (time == samples_.back().time) return;
      if (time < samples_.back().time || time - samples_.back().time > kMaxGapS)
        reset();
    }
    samples_.push_back({time, acceleration, angular_rate});
    // Keep one sample at/before the window boundary to measure its duration.
    while (samples_.size() > 1 && samples_[1].time <= time - kDurationS)
      samples_.pop_front();
    while (samples_.size() > kMaxSamples) samples_.pop_front();
  }

  ImuInitializationEstimate estimate() const {
    ImuInitializationEstimate result;
    result.samples = samples_.size();
    if (samples_.size() < 2) return result;
    result.duration_s = samples_.back().time - samples_.front().time;
    double rate_squared = 0.;
    for (const auto& sample : samples_) {
      result.acceleration += sample.acceleration;
      result.angular_rate += sample.angular_rate;
      rate_squared += sample.angular_rate.squaredNorm();
    }
    result.acceleration /= result.samples;
    result.angular_rate /= result.samples;
    for (const auto& sample : samples_) {
      result.acceleration_variance +=
          (sample.acceleration - result.acceleration).cwiseAbs2();
      result.angular_rate_variance +=
          (sample.angular_rate - result.angular_rate).cwiseAbs2();
    }
    result.acceleration_variance /= result.samples - 1;
    result.angular_rate_variance /= result.samples - 1;
    const double norm = result.acceleration.norm();
    if (norm < 1e-6) return result;
    result.acceleration_std_g = std::sqrt(result.acceleration_variance.sum()) / norm;
    result.angular_rate_rms = std::sqrt(rate_squared / result.samples);
    result.acceleration_mean_variance = result.acceleration_variance.sum() / result.samples;
    result.angular_rate_mean_variance = result.angular_rate_variance.sum() / result.samples;
    if (result.samples >= 4) {
      // Batch means retain correlated variation that variance/N would discount.
      // Compare scalar traces so the decision is invariant to sensor mounting rotation.
      std::array<Eigen::Vector3d, 4> acceleration_means, angular_rate_means;
      Eigen::Vector3d acceleration_center = Eigen::Vector3d::Zero();
      Eigen::Vector3d angular_rate_center = Eigen::Vector3d::Zero();
      for (size_t batch = 0; batch < 4; ++batch) {
        const size_t begin = batch * result.samples / 4;
        const size_t end = (batch + 1) * result.samples / 4;
        auto& a = acceleration_means[batch];
        auto& w = angular_rate_means[batch];
        a.setZero();
        w.setZero();
        for (size_t i = begin; i < end; ++i) {
          a += samples_[i].acceleration;
          w += samples_[i].angular_rate;
        }
        a /= end - begin;
        w /= end - begin;
        acceleration_center += a / 4.;
        angular_rate_center += w / 4.;
      }
      double acceleration_batch_variance = 0., angular_rate_batch_variance = 0.;
      for (size_t batch = 0; batch < 4; ++batch) {
        acceleration_batch_variance += (acceleration_means[batch] - acceleration_center).squaredNorm() / 12.;
        angular_rate_batch_variance += (angular_rate_means[batch] - angular_rate_center).squaredNorm() / 12.;
      }
      result.acceleration_mean_variance = std::max(result.acceleration_mean_variance, acceleration_batch_variance);
      result.angular_rate_mean_variance = std::max(result.angular_rate_mean_variance, angular_rate_batch_variance);
    }
    result.acceleration_mean_std_g = std::sqrt(result.acceleration_mean_variance) / norm;
    result.angular_rate_mean_std = std::sqrt(result.angular_rate_mean_variance);
    Eigen::Vector3d first_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d first_angular_rate = Eigen::Vector3d::Zero();
    const size_t half = result.samples / 2;
    for (size_t i = 0; i < half; ++i) {
      first_acceleration += samples_[i].acceleration;
      first_angular_rate += samples_[i].angular_rate;
    }
    const Eigen::Vector3d second_acceleration =
        (result.acceleration * result.samples - first_acceleration) / (result.samples - half);
    const Eigen::Vector3d second_angular_rate =
        (result.angular_rate * result.samples - first_angular_rate) / (result.samples - half);
    result.acceleration_mean_change_g =
        (first_acceleration / half - second_acceleration).norm() / norm;
    result.angular_rate_mean_change =
        (first_angular_rate / half - second_angular_rate).norm();
    result.ready = result.samples >= kMinSamples &&
                   result.duration_s + 1e-9 >= kDurationS &&
                   result.acceleration_mean_std_g <= kMaxAccelerationMeanStdG &&
                   result.angular_rate_mean_std <= kMaxAngularRateMeanStd &&
                   result.angular_rate_rms <= kMaxAngularRateRms &&
                   result.acceleration_mean_change_g <= kMaxAccelerationMeanChangeG &&
                   result.angular_rate_mean_change <= kMaxAngularRateMeanChange;
    return result;
  }
};
} // namespace fast_lio
