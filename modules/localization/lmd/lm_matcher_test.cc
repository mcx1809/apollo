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
#include "modules/localization/lmd/lm_matcher.h"
#include "gtest/gtest.h"
namespace apollo {
namespace localization {

class LMMatcherTest : public ::testing::Test {};
/*
TEST(LMMatcherTest, BestMatchScoreForTwoLane) {
  auto lm_provider = new LMProvider();
  auto lm_matcher = new LMMatcher(lm_provider);
  const apollo::perception::LaneMarkers lane_markers;
  EXPECT_TRUE(lane_markers.has_left_lane_marker());
  const auto& lane_maker_per = lane_markers.left_lane_marker();

  apollo::common::PointENU position;
  position.set_x(6882291.98649);
  position.set_y(3111354.33775);
  position.set_z(65.44452);

  lm_provider->FindNearestLaneMarkerIndex(position);

  std::pair<int, int> result_index = lm_provider->GetCurrentLaneMarkerIndex();
  EXPECT_NE(std::numeric_limits<int>::max(), result_index.first);
  EXPECT_NE(std::numeric_limits<int>::max(), result_index.second);
  apollo::localization::OdometryLaneMarker lane_marker_map =
      lm_provider->GetLaneMarker();
  EXPECT_TRUE(lane_marker_map.has_start_position());
  EXPECT_TRUE(lane_marker_map.has_end_position());

  double d_position = 0.0;
  double d_score =
      BestMatchScoreForTwoLane(lane_maker_per, lane_marker_map, d_position);

  delete lm_provider;
  delete lm_matcher;
}*/
}  // namespace localization
}  // namespace apollo