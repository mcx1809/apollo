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

#include <limits>
#include <utility>

#include "modules/common/log.h"
#include "modules/common/math/vec2d.h"

namespace apollo {
namespace localization {

using apollo::common::Point3D;
using apollo::common::PointENU;
using apollo::common::Status;
using apollo::common::math::RotateAxis;
using apollo::common::math::Vec2d;
using apollo::perception::LaneMarker;
using apollo::perception::LaneMarkers;

namespace {
constexpr double kMapResolution = 0.01;
constexpr char kMapSizeMaxLevel = 32;
}  // namespace

PCMap::PCMap(LMProvider* provider) {
  CHECK_NOTNULL(provider);

  // initialize root node
  nodes_.resize(1);
  auto& root = nodes_[0];
  root.level = kMapSizeMaxLevel;

  // load all of data from provider
  provider_ = provider;
  auto pack_size = provider_->GetLaneMarkerPackSize();
  for (decltype(pack_size) pack_index = 0; pack_index < pack_size;
       ++pack_index) {
    auto lane_marker_size = provider_->GetLaneMarkerSize(pack_index);
    for (decltype(lane_marker_size) lane_index = 0;
         lane_index < lane_marker_size; ++lane_index) {
      auto lane_marker =
          provider_->GetLaneMarker(std::make_pair(pack_index, lane_index));
      if (lane_marker != nullptr) LoadLaneMarker(*lane_marker);
    }
  }
}

Status PCMap::UpdateRange(const PointENU& position, double radius) {
  return Status::OK();
}

const PCMapPoint* PCMap::GetNearestPoint(const PointENU& position,
                                         double* d2) const {
  auto node_index = MakeNodeIndex(position.x(), position.y());

  auto node_it = nodes_.find(node_index);
  if (node_it == nodes_.end()) return nullptr;

  auto d2_min = std::numeric_limits<double>::max();
  const PCMapPoint* nearest_point = nullptr;
  for (const auto& point_pair : node_it->second.points) {
    const auto& point = point_pair.second;
    auto d2_testing = Vec2d(point.position.x(), point.position.y())
                          .DistanceSquareTo(Vec2d(position.x(), position.y()));
    if (d2_testing < d2_min) {
      d2_min = d2_testing;
      nearest_point = &point;
    }
  }

  if (d2 != nullptr) *d2 = d2_min;

  return nearest_point;
}

const PCMapPoint* PCMap::GetNearestPoint(const PointENU& position) const {
  return GetNearestPoint(position, nullptr);
}

void PCMap::LoadLaneMarker(const OdometryLaneMarker& lane_marker) {
  PCMapPoint* seg_point = nullptr;

  for (const auto& lane_point : lane_marker.points()) {
    auto node_index =
        MakeNodeIndex(lane_point.position().x(), lane_point.position().y());

    auto set_point = [&](std::map<Index2D, PCMapPoint>* points) {
      Index2D index;
      index.x = lane_point.position().x() / kMapResolution;
      index.y = lane_point.position().y() / kMapResolution;

      auto point_it = points->find(index);
      if (point_it == points->end()) {
        auto& point =
            points->emplace(index, PCMapPoint(lane_point)).first->second;
        if (seg_point != nullptr) {
          seg_point->next = &point;
          point.prev = seg_point;
        }
        seg_point = &point;
      } else {
        auto& point = point_it->second;
        if (seg_point == nullptr) {
          if (point.next == nullptr) seg_point = &point;
        } else {
          if (point.prev == nullptr) {
            seg_point->next = &point;
            point.prev = seg_point;
          }
          if (point.next == nullptr)
            seg_point = &point;
          else
            seg_point = nullptr;
        }
      }
    };

    auto node_it = nodes_.find(node_index);
    if (node_it == nodes_.end())
      set_point(&nodes_[node_index].points);
    else
      set_point(&node_it->second.points);
  }
}

const OdometryLaneMarker PCMap::GenerateOdometryLaneMarker(
    const LaneMarker& lanemarker, const PointENU position, const double heading,
    const double lane_length, const int point_number) const {
  OdometryLaneMarker odo_lane;
  for (int i = 0; i < point_number; ++i) {
    auto point = odo_lane.add_points();
    auto relative_x = lane_length / point_number * i;
    auto relative_y = CalCurveValue(
        relative_x, lanemarker.c0_position(), lanemarker.c1_heading_angle(),
        lanemarker.c2_curvature(), lanemarker.c3_curvature_derivative());
    auto curvature_value = CalCurvity(
        relative_x, lanemarker.c0_position(), lanemarker.c1_heading_angle(),
        lanemarker.c2_curvature(), lanemarker.c3_curvature_derivative());
    point->set_curvature(curvature_value);
    double enu_x, enu_y;
    RotateAxis(-heading, relative_x, relative_y, &enu_x, &enu_y);
    point->mutable_position()->set_x(position.x() + enu_x);
    point->mutable_position()->set_y(position.y() + enu_y);
    point->mutable_position()->set_z(position.z());
    double enu_x_direct, enu_y_direct;
    RotateAxis(-heading, 0, 1, &enu_x_direct, &enu_y_direct);
    point->mutable_direct()->set_x(enu_x_direct);
    point->mutable_direct()->set_y(enu_y_direct);
    point->mutable_direct()->set_z(0.0);
  }
  return odo_lane;
}

std::vector<OdometryLaneMarker> PCMap::PrepareLaneMarkers(
    const LaneMarkers& source, const PointENU position, const double heading,
    const double lane_length, const int point_number) {
  std::vector<OdometryLaneMarker> result;
  if (source.has_left_lane_marker()) {
    const auto& lanemarker = source.left_lane_marker();
    const auto& odo_lane_marker = GenerateOdometryLaneMarker(
        lanemarker, position, heading, lane_length, point_number);
    result.emplace_back(odo_lane_marker);
  }
  if (source.has_right_lane_marker()) {
    const auto& lanemarker = source.right_lane_marker();
    const auto& odo_lane_marker = GenerateOdometryLaneMarker(
        lanemarker, position, heading, lane_length, point_number);
    result.emplace_back(odo_lane_marker);
  }

  for (int i = 0; i < source.next_left_lane_marker_size(); ++i) {
    const auto& lanemarker = source.next_left_lane_marker(i);
    const auto& odo_lane_marker = GenerateOdometryLaneMarker(
        lanemarker, position, heading, lane_length, point_number);
    result.emplace_back(odo_lane_marker);
  }

  for (int i = 0; i < source.next_right_lane_marker_size(); ++i) {
    const auto& lanemarker = source.next_right_lane_marker(i);
    const auto& odo_lane_marker = GenerateOdometryLaneMarker(
        lanemarker, position, heading, lane_length, point_number);
    result.emplace_back(odo_lane_marker);
  }
  return result;
}

double PCMap::CalCurveValue(const double x_value, const double c0,
                            const double c1, const double c2,
                            const double c3) const {
  return c3 * pow(x_value, 3.0) + c2 * pow(x_value, 2.0) + c1 * x_value + c0;
}

double PCMap::CalDerivative(const double x_value, const double c0,
                            const double c1, const double c2,
                            const double c3) const {
  return 3 * c3 * pow(x_value, 2.0) + 2 * c2 + c1;
}

double PCMap::CalCurvity(const double x_value, const double c0, const double c1,
                         const double c2, const double c3) const {
  double derivative = CalDerivative(x_value, c0, c1, c2, c3);
  return abs(6 * c3 * x_value + 2 * c2) /
         pow(1 + pow(derivative, 2.0), (3.0 / 2));
}

std::size_t PCMap::InsertPoint(std::size_t point_index) {
  const auto& point = points_[point_index];
  auto px = (long long)(point.position.x() / kMapResolution);
  auto py = (long long)(point.position.y() / kMapResolution);

  long long cur_cent_x = 0;
  long long cur_cent_y = 0;
  long long cur_half_size = 1LL << (kMapSizeMaxLevel - 1);

  if (px < cur_cent_x - cur_half_size || px > cur_cent_x + cur_half_size ||
      py < cur_cent_y - cur_half_size || py > cur_cent_y + cur_half_size)
    return (std::size_t)-1;

  auto insert_point = [&](std::size_t node_index) {

  };

  insert_point
}

std::size_t PCMap::FetchPoint() {
  if (free_point_head_ != (std::size_t)-1) {
    auto index = free_point_head_;
    auto& point = points_[index];
    free_point_head_ = point.next;
    point.next = (std::size_t)-1;
    return index;
  } else {
    points_.resize(points_.size() + 1);
    return points_.size() - 1;
  }
}

void PCMap::StorePoint(std::size_t index) {
  auto& point = points_[index];
  point.prev = (std::size_t)-1;
  point.next = free_point_head_;
  free_point_head_ = index;
}

std::size_t PCMap::FetchNode() {
  if (free_node_head_ != (std::size_t)-1) {
    auto index = free_node_head_;
    auto& node = nodes_[index];
    free_node_head_ = node.next;
    node.next = (std::size_t)-1;
    return index;
  } else {
    nodes_.resize(nodes_.size() + 1);
    return nodes_.size() - 1;
  }
}

void PCMap::StoreNode(std::size_t index) {
  auto& node = nodes_[index];
  node.next = free_node_head_;
  free_node_head_ = index;
}

}  // namespace localization
}  // namespace apollo
