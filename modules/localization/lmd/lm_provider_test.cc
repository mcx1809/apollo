/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/localization/lmd/lm_provider.h"
#include "gtest/gtest.h"
namespace apollo {
namespace localization {

class LMProviderTest : public ::testing::Test {};

TEST(LMProviderTest, GetCurrentLaneMarkerIndex) {
  auto lm_provider_ = new LMProvider();
  std::pair<int, int> result = lm_provider_->GetCurrentLaneMarkerIndex();
  EXPECT_EQ(std::numeric_limits<int>::max(), result.first);
  EXPECT_EQ(std::numeric_limits<int>::max(), result.second);
  delete lm_provider_;
}

TEST(LMProviderTest, FindNearestLaneMarkerIndex) {
  apollo::common::PointENU position;
  position.set_x(6882291.98649);
  position.set_y(3111354.33775);
  position.set_z(65.44452);
  auto lm_provider_ = new LMProvider();
  lm_provider_->FindNearestLaneMarkerIndex(position);
  std::pair<int, int> result = lm_provider_->GetCurrentLaneMarkerIndex();
  EXPECT_NE(std::numeric_limits<int>::max(), result.first);
  EXPECT_NE(std::numeric_limits<int>::max(), result.second);
  delete lm_provider_;
}

TEST(LMProviderTest, GetPrevLaneMarkerIndex) {
  apollo::common::PointENU position;
  position.set_x(6882291.98649);
  position.set_y(3111354.33775);
  position.set_z(65.44452);
  auto lm_provider_ = new LMProvider();
  lm_provider_->FindNearestLaneMarkerIndex(position);
  std::pair<int, int> before_action_index =
      lm_provider_->GetCurrentLaneMarkerIndex();
  lm_provider_->GetPrevLaneMarkerIndex();
  std::pair<int, int> after_action_index =
      lm_provider_->GetCurrentLaneMarkerIndex();
  EXPECT_EQ(after_action_index.first, before_action_index.first);
  EXPECT_EQ(after_action_index.second, before_action_index.second - 1);
  delete lm_provider_;
}

TEST(LMProviderTest, GetNextLaneMarkerIndex) {
  apollo::common::PointENU position;
  position.set_x(6882291.98649);
  position.set_y(3111354.33775);
  position.set_z(65.44452);
  auto lm_provider_ = new LMProvider();
  lm_provider_->FindNearestLaneMarkerIndex(position);
  std::pair<int, int> before_action_index =
      lm_provider_->GetCurrentLaneMarkerIndex();
  lm_provider_->GetNextLaneMarkerIndex();
  std::pair<int, int> after_action_index =
      lm_provider_->GetCurrentLaneMarkerIndex();
  EXPECT_EQ(after_action_index.first, before_action_index.first);
  EXPECT_EQ(after_action_index.second, before_action_index.second + 1);
}

TEST(LMProviderTest, GetLeftLaneMarkerIndex) {
  apollo::common::PointENU position;
  position.set_x(6882291.98649);
  position.set_y(3111354.33775);
  position.set_z(65.44452);
  auto lm_provider_ = new LMProvider();
  lm_provider_->FindNearestLaneMarkerIndex(position);
  std::pair<int, int> before_action_index =
      lm_provider_->GetCurrentLaneMarkerIndex();
  lm_provider_->GetLeftLaneMarkerIndex();
  std::pair<int, int> after_action_index =
      lm_provider_->GetCurrentLaneMarkerIndex();
  EXPECT_EQ(after_action_index.first, before_action_index.first - 1);
  EXPECT_EQ(after_action_index.second, before_action_index.second);
}

TEST(LMProviderTest, GetRightLaneMarkerIndex) {
  apollo::common::PointENU position;
  position.set_x(6882291.98649);
  position.set_y(3111354.33775);
  position.set_z(65.44452);
  auto lm_provider_ = new LMProvider();
  lm_provider_->FindNearestLaneMarkerIndex(position);
  std::pair<int, int> before_action_index =
      lm_provider_->GetCurrentLaneMarkerIndex();
  lm_provider_->GetLeftLaneMarkerIndex();
  std::pair<int, int> after_action_index =
      lm_provider_->GetCurrentLaneMarkerIndex();
  EXPECT_EQ(after_action_index.first + 1, before_action_index.first);
  EXPECT_EQ(after_action_index.second, before_action_index.second);
}

TEST(LMProviderTest, GetLaneMarker) {
  apollo::common::PointENU position;
  position.set_x(6882291.98649);
  position.set_y(3111354.33775);
  position.set_z(65.44452);
  auto lm_provider_ = new LMProvider();
  lm_provider_->FindNearestLaneMarkerIndex(position);
  std::pair<int, int> result_index = lm_provider_->GetCurrentLaneMarkerIndex();
  EXPECT_NE(std::numeric_limits<int>::max(), result_index.first);
  EXPECT_NE(std::numeric_limits<int>::max(), result_index.second);
  apollo::localization::OdometryLaneMarker lane_marker =
      lm_provider_->GetLaneMarker();
  EXPECT_TRUE(lane_marker.has_start_position());
  EXPECT_TRUE(lane_marker.has_end_position());
}
}  // namespace localization
}  // namespace apollo
