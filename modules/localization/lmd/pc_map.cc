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
using apollo::common::math::Vec2d;

namespace {
constexpr double kNodeSize = 1.0;
constexpr double kMapResolution = 0.05;
}  // namespace

PCMap::PCMap(LMProvider* provider) {
  CHECK_NOTNULL(provider);
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

PCMap::Index2D PCMap::MakeNodeIndex(double x, double y) const {
  Index2D index;
  index.x = x / kNodeSize;
  index.y = y / kNodeSize;
  return index;
}

}  // namespace localization
}  // namespace apollo
