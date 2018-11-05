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

#include "modules/localization/lmd/lm_bag_processor.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {

TEST(LMProcessorTest, SerializeToFile) {
  LMProcessor* processor_ = new LMProcessor(FLAGS_lmd_rawinput_bag_file);
  OdometryLaneMarkersPack lane_marker_pack_;
  processor_->PrepareMarkersPack(&lane_marker_pack_);
  EXPECT_EQ(2, lane_marker_pack_.lane_markers_size());

  auto lane_markers_group =
      lane_marker_pack_.lane_markers(lane_marker_pack_.lane_markers_size() - 1);
  EXPECT_EQ(1442, lane_markers_group.lane_marker_size());
  auto lane_marker =
      lane_markers_group.lane_marker(lane_markers_group.lane_marker_size() / 2);
  EXPECT_EQ(10, lane_marker.points_size());
  auto point = lane_marker.points(lane_marker.points_size() / 5);
  EXPECT_TRUE(point.has_position());
  EXPECT_TRUE(point.has_direct());
  EXPECT_TRUE(point.has_curvature());
  auto location = point.position();
  EXPECT_NEAR(681654.02825360361, location.x(), 0.001);
  EXPECT_NEAR(3112401.4922517287, location.y(), 0.001);
  EXPECT_NEAR(66.44058248098932, location.z(), 0.001);
  auto direction = point.direct();
  EXPECT_NEAR(-0.26960741335376043, direction.x(), 0.001);
  EXPECT_NEAR(0.96297032283694728, direction.y(), 0.001);
  EXPECT_NEAR(0.0, direction.z(), 0.001);
  EXPECT_NEAR(0.0, point.curvature(), 0.001);

  auto lane_markers_group_next = lane_marker_pack_.lane_markers(0);
  EXPECT_EQ(1442, lane_markers_group_next.lane_marker_size());
  auto lane_marker_next = lane_markers_group_next.lane_marker(
      lane_markers_group_next.lane_marker_size() - 1);
  EXPECT_EQ(10, lane_marker_next.points_size());
  auto point_next = lane_marker_next.points(lane_marker_next.points_size() / 5);
  EXPECT_TRUE(point_next.has_position());
  EXPECT_TRUE(point_next.has_direct());
  EXPECT_TRUE(point_next.has_curvature());
  auto location_next = point_next.position();
  EXPECT_NEAR(683090.56203492463, location_next.x(), 0.001);
  EXPECT_NEAR(3110709.2882031966, location_next.y(), 0.001);
  EXPECT_NEAR(57.115302797824704, location_next.z(), 0.001);
  auto direction_next = point_next.direct();
  EXPECT_NEAR(-0.79389947655760185, direction_next.x(), 0.001);
  EXPECT_NEAR(0.60804902855079523, direction_next.y(), 0.001);
  EXPECT_NEAR(0.0, direction_next.z(), 0.001);
  EXPECT_NEAR(0.0, point_next.curvature(), 0.001);
}

}  // namespace localization
}  // namespace apollo
