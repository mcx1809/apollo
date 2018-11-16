/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <iomanip>

#include "modules/localization/lmd/predictor/filter/predictor_filtered_imu.h"

#include "modules/common/log.h"

namespace apollo {
namespace localization {

using apollo::common::Point3D;
using apollo::common::Status;

namespace {
constexpr double kSamplingInterval = 0.01;
}  // namespace

PredictorFilteredImu::PredictorFilteredImu(double memory_cycle_sec)
    : Predictor(memory_cycle_sec) {
  name_ = kPredictorFilteredImuName;
  dep_predicteds_.emplace(kPredictorImuName, PoseList(memory_cycle_sec));
  on_adapter_thread_ = true;

  constexpr double kCutoffFreq = 20.0;
  InitLPFilter(kCutoffFreq);
}

PredictorFilteredImu::~PredictorFilteredImu() {}

bool PredictorFilteredImu::UpdateChassis(
    const apollo::canbus::Chassis& chassis) {
  if (!chassis.has_header() || !chassis.header().has_timestamp_sec() ||
      !chassis.has_speed_mps()) {
    AERROR << "Message has not some feilds";
    return false;
  }

  auto timestamp_sec = chassis.header().timestamp_sec();
  if (!chassis_.Push(timestamp_sec, chassis.speed_mps())) {
    AWARN << std::setprecision(15)
          << "Failed push speed_mps to list, with timestamp[" << timestamp_sec
          << "]";
    return false;
  }

  return true;
}

bool PredictorFilteredImu::Updateable() const {
  const auto& imu = dep_predicteds_.find(kPredictorImuName)->second;
  if (predicted_.empty()) {
    return !imu.empty();
  } else {
    return !predicted_.Newer(imu.Latest()->first - kSamplingInterval);
  }
}

Status PredictorFilteredImu::Update() {
  LPFilter();
  return Status::OK();
}

void PredictorFilteredImu::ResamplingFilter() {
  const auto& imu = dep_predicteds_[kPredictorImuName];
  auto latest_it = predicted_.Latest();
  if (latest_it == predicted_.end()) {
    predicted_.Push(imu.begin()->first, imu.begin()->second);
  } else {
    auto timestamp_sec = latest_it->first + kSamplingInterval;
    Pose pose;
    imu.FindNearestPose(timestamp_sec, &pose);
    predicted_.Push(timestamp_sec, pose);
  }
}

void PredictorFilteredImu::InitLPFilter(double cutoff_freq) {
  auto omega = 2.0 * M_PI * cutoff_freq * kSamplingInterval;
  auto sin_o = std::sin(omega);
  auto cos_o = std::cos(omega);
  auto alpha = sin_o / (2.0 / std::sqrt(2));
  iir_filter_bz_[0] = (1.0 - cos_o) / 2.0;
  iir_filter_bz_[1] = 1.0 - cos_o;
  iir_filter_bz_[2] = (1.0 - cos_o) / 2.0;
  iir_filter_az_[0] = 1.0 + alpha;
  iir_filter_az_[1] = -2.0 * cos_o;
  iir_filter_az_[2] = 1.0 - alpha;
}

void PredictorFilteredImu::LPFilter() {
  double timestamp_sec;
  Point3D x_0, x_1, x_2;
  Point3D y_0, y_1, y_2;
  Pose pose;

  const auto& imu = dep_predicteds_[kPredictorImuName];
  auto latest_it = predicted_.Latest();
  if (latest_it == predicted_.end()) {
    timestamp_sec = imu.begin()->first;

    x_0 = x_1 = x_2 = imu.begin()->second.linear_acceleration();
    y_1 = y_2 = x_2;
    pose = imu.begin()->second;
  } else {
    timestamp_sec = latest_it->first + kSamplingInterval;

    imu.FindNearestPose(timestamp_sec - 2.0 * kSamplingInterval, &pose);
    x_2 = pose.linear_acceleration();
    imu.FindNearestPose(timestamp_sec - kSamplingInterval, &pose);
    x_1 = pose.linear_acceleration();
    imu.FindNearestPose(timestamp_sec, &pose);
    x_0 = pose.linear_acceleration();

    if (predicted_.size() == 1) {
      y_2 = x_2;
    } else {
      auto it_2 = latest_it;
      it_2--;
      y_2 = it_2->second.linear_acceleration();
    }
    y_1 = latest_it->second.linear_acceleration();
  }

  //  two-order Butterworth filter
  auto but_filter = [&](double x_0, double x_1, double x_2, double y_1,
                        double y_2) {
    const auto& bz = iir_filter_bz_;
    const auto& az = iir_filter_az_;
    return (bz[0] * x_0 + bz[1] * x_1 + bz[2] * x_2 - az[1] * y_1 -
            az[2] * y_2) /
           az[0];
  };

  y_0.set_x(but_filter(x_0.x(), x_1.x(), x_2.x(), y_1.x(), y_2.x()));
  y_0.set_y(but_filter(x_0.y(), x_1.y(), x_2.y(), y_1.y(), y_2.y()));
  y_0.set_z(but_filter(x_0.z(), x_1.z(), x_2.z(), y_1.z(), y_2.z()));
  pose.mutable_linear_acceleration()->CopyFrom(y_0);
  predicted_.Push(timestamp_sec, pose);
}

}  // namespace localization
}  // namespace apollo
