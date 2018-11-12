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

#include "modules/localization/lmd/predictor/output/predictor.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {

using apollo::common::Status;

class PredictorOutputTest : public ::testing::Test {};

TEST_F(PredictorOutputTest, PredictByImu) {
  PredictorOutput predictor(
      10.0,
      [&](const LocalizationEstimate& localization) { return Status::OK(); });

  auto& imu = predictor.dep_predicteds_[kPredictorImuName];
  Pose imu_pose;
  imu_pose.mutable_linear_acceleration()->set_x(1.0);
  imu_pose.mutable_linear_acceleration()->set_y(1.0);
  imu_pose.mutable_linear_acceleration()->set_z(0.0);
  imu_pose.mutable_angular_velocity()->set_x(0.0);
  imu_pose.mutable_angular_velocity()->set_y(0.0);
  imu_pose.mutable_angular_velocity()->set_z(0.0);
  imu.Push(0.0, imu_pose);
  imu.Push(3.0, imu_pose);

  Pose old_pose;
  old_pose.mutable_position()->set_x(0.0);
  old_pose.mutable_position()->set_y(0.0);
  old_pose.mutable_position()->set_z(0.0);
  old_pose.mutable_orientation()->set_qw(1.0);
  old_pose.mutable_orientation()->set_qx(0.0);
  old_pose.mutable_orientation()->set_qy(0.0);
  old_pose.mutable_orientation()->set_qz(0.0);
  old_pose.mutable_linear_velocity()->set_x(0.0);
  old_pose.mutable_linear_velocity()->set_y(0.0);
  old_pose.mutable_linear_velocity()->set_z(0.0);

  Pose new_pose;
  EXPECT_TRUE(predictor.PredictByImu(0.0, old_pose, 1.0, &new_pose));
}

}  // namespace localization
}  // namespace apollo
