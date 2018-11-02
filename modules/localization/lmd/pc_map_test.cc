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

class PCMapTest : public ::testing::Test {};

TEST_F(PCMapTest, FetchAndStore) {
  PCMap map;

  auto point_index0 = map.FetchPoint();
  EXPECT_EQ(1, map.points_.size());
  auto point_index1 = map.FetchPoint();
  EXPECT_EQ(2, map.points_.size());
  auto point_index2 = map.FetchPoint();
  EXPECT_EQ(3, map.points_.size());
  auto point_index3 = map.FetchPoint();
  EXPECT_EQ(4, map.points_.size());
  auto point_index4 = map.FetchPoint();
  EXPECT_EQ(5, map.points_.size());
  map.StorePoint(point_index0);
  point_index0 = map.FetchPoint();
  EXPECT_EQ(5, map.points_.size());
  map.StorePoint(point_index1);
  map.StorePoint(point_index2);
  map.StorePoint(point_index3);
  map.StorePoint(point_index4);
  EXPECT_EQ(point_index4, map.FetchPoint());
  EXPECT_EQ(point_index3, map.FetchPoint());

  auto node_index0 = map.FetchNode();
  EXPECT_EQ(2, map.nodes_.size());
  auto node_index1 = map.FetchNode();
  EXPECT_EQ(3, map.nodes_.size());
  auto node_index2 = map.FetchNode();
  EXPECT_EQ(4, map.nodes_.size());
  auto node_index3 = map.FetchNode();
  EXPECT_EQ(5, map.nodes_.size());
  auto node_index4 = map.FetchNode();
  EXPECT_EQ(6, map.nodes_.size());
  map.StoreNode(node_index0);
  node_index0 = map.FetchNode();
  EXPECT_EQ(6, map.nodes_.size());
  map.StoreNode(node_index1);
  map.StoreNode(node_index2);
  map.StoreNode(node_index3);
  map.StoreNode(node_index4);
  EXPECT_EQ(node_index4, map.FetchNode());
  EXPECT_EQ(node_index3, map.FetchNode());
}

TEST_F(PCMapTest, InsertPoint) {
  PCMap map;

  auto point_index = map.FetchPoint();
  auto& point = map.points_[point_index];
  point.position.set_x(100.0);
  point.position.set_y(100.0);
  auto node_index = map.InsertPoint(0, point_index);
  EXPECT_EQ(0, node_index);
  const auto& node = map.nodes_[node_index];
  EXPECT_EQ(true, node.IsPoint(1));
  EXPECT_EQ((PCMapIndex)-1, node.c_index[0]);
  EXPECT_EQ(point_index, node.c_index[1]);
  EXPECT_EQ((PCMapIndex)-1, node.c_index[2]);
  EXPECT_EQ((PCMapIndex)-1, node.c_index[3]);

  auto point_index1 = map.FetchPoint();
  auto& point1 = map.points_[point_index1];
  point1.position.set_x(200.0);
  point1.position.set_y(100.0);
  auto node_index1 = map.InsertPoint(0, point_index1);
  EXPECT_EQ(false, node.IsPoint(1));
  EXPECT_EQ((PCMapIndex)-1, node.c_index[0]);
  EXPECT_EQ(node_index1, node.c_index[1]);
  EXPECT_EQ((PCMapIndex)-1, node.c_index[2]);
  EXPECT_EQ((PCMapIndex)-1, node.c_index[3]);
  const auto& node1 = map.nodes_[node_index1];
  EXPECT_EQ(true, node1.IsPoint(2));
  EXPECT_EQ(true, node1.IsPoint(3));
  EXPECT_EQ((PCMapIndex)-1, node1.c_index[0]);
  EXPECT_EQ(point_index, node1.c_index[2]);
  EXPECT_EQ(point_index1, node1.c_index[3]);
}

TEST_F(PCMapTest, FindNearestPointInNode) {
  PCMap map;

  auto src_point_index = map.FetchPoint();
  auto& src_point = map.points_[src_point_index];
  src_point.position.set_x(100.0);
  src_point.position.set_y(100.0);
  map.InsertPoint(0, src_point_index);

  double x = 0.0;
  double y = 0.0;
  PCMapIndex node_index;
  PCMapIndex point_index;
  double d2;
  bool on_boundary;
  /*std::tie(node_index, point_index, d2, on_boundary) =
      map.FindNearestPointInNode(0, map.GetMapX(x), map.GetMapY(y), x, y);
  EXPECT_EQ(src_point_index, point_index);
  EXPECT_NEAR(20000.0, d2, 1e-3);
  EXPECT_TRUE(on_boundary);*/

  auto src_point_index1 = map.FetchPoint();
  auto& src_point1 = map.points_[src_point_index1];
  src_point1.position.set_x(200.0);
  src_point1.position.set_y(100.0);
  map.InsertPoint(0, src_point_index1);

  x = 190.0;
  y = 100.0;
  std::tie(node_index, point_index, d2, on_boundary) =
      map.FindNearestPointInNode(0, map.GetMapX(x), map.GetMapY(y), x, y);
  EXPECT_TRUE(false);
  /*EXPECT_EQ(src_point_index1, point_index);
 EXPECT_NEAR(100.0, d2, 1e-3);
 EXPECT_TRUE(on_boundary);*/
}

/*TEST(PCMapTest, GetNearestPoint) {
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
}*/

}  // namespace localization
}  // namespace apollo
