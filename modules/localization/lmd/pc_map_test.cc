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

#include "modules/localization/lmd/pc_map.h"
#include "modules/localization/lmd/lm_provider.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace {
constexpr int kPointsNumInsertToMap = 240;
constexpr double kInsertMapLaneLength = 12.0;
}  // namespace
using apollo::common::PointENU;

TEST(PCMapTest, GetNearestPoint) {
  LMProvider provider;
  PCMap map(&provider);
  PointENU position;
  position.set_x(683092.0);
  position.set_y(3110712.0);
  position.set_z(57.0);
  map.UpdateRange(position, 16.0);

  PointENU search_point;
  search_point.set_x(683092.0);
  search_point.set_y(3110712.0);
  search_point.set_z(57.0);
  auto nearest_point = map.Point(map.GetNearestPoint(search_point));
  EXPECT_NE(nullptr, nearest_point);
  if (nearest_point != nullptr) {
    EXPECT_NEAR(683092.31, nearest_point->position.x(), 0.01);
    EXPECT_NEAR(3110712.43, nearest_point->position.y(), 0.01);
    EXPECT_NEAR(57.08, nearest_point->position.z(), 0.01);
  }
}

TEST(PCMapTest, PrepareLaneMarkers) {
  LMProvider provider;
  PCMap map(&provider);
  PointENU position;
  position.set_x(681732.77703);
  position.set_y(3112136.72507);
  position.set_z(60.723);
  apollo::perception::LaneMarker left_lane_marker;
  left_lane_marker.set_c0_position(1.48438);
  left_lane_marker.set_c1_heading_angle(-0.00586);
  left_lane_marker.set_c2_curvature(0.00031);
  left_lane_marker.set_c3_curvature_derivative(0.000);
  apollo::perception::LaneMarker right_lane_marker;
  right_lane_marker.set_c0_position(-2.10156);
  right_lane_marker.set_c1_heading_angle(0.00488);
  right_lane_marker.set_c2_curvature(0.00008);
  right_lane_marker.set_c3_curvature_derivative(0.000);
  apollo::perception::LaneMarkers lane_markers;
  lane_markers.mutable_left_lane_marker()->CopyFrom(left_lane_marker);
  lane_markers.mutable_right_lane_marker()->CopyFrom(right_lane_marker);
  double heading = -1.19396;
  auto source_lanes =
      map.PrepareLaneMarkers(lane_markers, position, heading,
                             kInsertMapLaneLength, kPointsNumInsertToMap);
  EXPECT_EQ(2, source_lanes.size());
  for (auto lane : source_lanes) {
    EXPECT_EQ(kPointsNumInsertToMap, lane.points_size());
    map.LoadLaneMarker(lane);
  }
}
}  // namespace localization
}  // namespace apollo
