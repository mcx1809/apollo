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

static long long GetMapX(double x) { return (long long)(x / kMapResolution); }

static long long GetMapY(double y) { return (long long)(y / kMapResolution); }

PCMap::PCMap(LMProvider* provider) {
  CHECK_NOTNULL(provider);

  // initialize root node
  nodes_.resize(1);
  auto& root = nodes_[0];
  root.level = kMapSizeMaxLevel;
  root.cx = 0;
  root.cy = 0;

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
  auto seg_point_index = (std::size_t)-1;
  std::size_t last_node_index = 0;
  for (const auto& lane_point : lane_marker.points()) {
    auto point_index = FetchPoint();
    auto& point = points_[point_index];
    point.Set(lane_point);

    last_node_index = InsertPoint(last_node_index, point_index);
    if (last_node_index == (std::size_t)-1) {
      StorePoint(point_index);
      seg_point_index = (std::size_t)-1;
      last_node_index = 0;
      continue;
    }

    if (seg_point_index == (std::size_t)-1) {
      if (point.next == (std::size_t)-1) seg_point_index = point_index;
    } else {
      if (point.prev == (std::size_t)-1) {
        point.prev = seg_point_index;
        auto& seg_point = points_[seg_point_index];
        seg_point.next = point_index;
      }
      if (point.next == (std::size_t)-1)
        seg_point_index = point_index;
      else
        seg_point_index = (std::size_t)-1;
    }
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

std::size_t InsertPoint(std::size_t node_index, std::size_t point_index) {
  const auto& point = points_[point_index];
  auto px = GetMapX(point.position.x());
  auto py = GetMapY(point.position.y());
  return InsertPoint(node_index, point_index, px, py);
}

std::size_t PCMap::InsertPoint(std::size_t node_index, std::size_t point_index,
                               long long px, long long py) {
  const auto& node = nodes_[node_index];
  if (node.OnBoundary(px, py)) {
    InsertPointInNode(node_index, point_index, px, py);
  } else {
    if (node.p_index != (std::size_t)-1)
      return InsertPoint(node.p_index, point_index, px, py);
    else
      return (std::size_t)-1;
  }
}

std::size_t PCMap::InsertPointInNode(std::size_t node_index,
                                     std::size_t point_index, long long px,
                                     long long py) {
  auto& node = nodes_[node_index];
  auto c_pos = node.GetPos(px, py);
  auto c_index = node.c_index[c_pos];
  if (c_index == (std::size_t)-1) {
    node.SetPoint(c_pos, point_index);
    return node_index;
  }

  if (node.IsPoint(c_pos)) {
    auto cur_point_index = c_index;
    const auto& cur_point = points_[cur_point_index];
    auto cur_px = GetMapX(cur_point.position.x());
    auto cur_py = GetMapY(cur_point.position.y());

    auto m_index = FetchNode();
    auto& m_node = nodes_[m_index];
    auto m_pos = c_pos;
    m_node.SetParentNode(node_index);
    node.SetChildNode(m_pos, m_index);
    m_node.cx = node.cx;
    m_node.cy = node.cy;
    m_node.level = node.level;
    while (m_node.level) {
      m_node.level--;
      m_node.SetCXY(m_node.cx, m_node.cy, m_pos);
      auto p_pos = m_node.GetPos(px, py);
      auto cp_pos = m_node.GetPos(cur_px, cur_py);
      if (cp_pos != p_pos) {
        m_node.SetPoint(p_pos, point_index);
        m_node.SetPoint(cp_pos, cur_point_index);
        return m_index;
      }
    }

    auto& point = points_[point_index];
    point.prev = cur_point.prev;
    point.next = cur_point.next;
    if (point.prev != (std::size_t)-1) {
      auto& prev_point = points_[point.prev];
      prev_point.next = point_index;
    }
    if (point.next != (std::size_t)-1) {
      auto& next_point = points_[point.next];
      next_point.prev = point_index;
    }
    StorePoint(cur_point_index);
    return m_index;
  } else {
    auto& c_node = nodes_[c_index];
    if (c_node.OnBoundary(px, py))
      return InsertPointInNode(c_index, point_index, px, py);

    auto m_index = FetchNode();
    auto& m_node = nodes_[m_index];
    auto m_pos = c_pos;
    m_node.SetParentNode(node_index);
    node.SetChildNode(m_pos, m_index);
    m_node.cx = node.cx;
    m_node.cy = node.cy;
    m_node.level = node.level;
    while (true) {
      m_node.level--;
      m_node.SetCXY(m_node.cx, m_node.cy, m_pos);
      auto c_pos = m_node.GetPos(c_node.cx, c_node.cy);
      auto p_pos = m_node.GetPos(px, py);
      if (c_pos != p_pos) {
        c_node.SetParentNode(m_index);
        m_node.SetChildNode(c_pos, c_index);
        m_node.SetPoint(p_pos, point_index);
        return m_index;
      }
    }
  }
}

std::tuple<std::size_t, std::size_t, double> PCMap::FindNearestPointInNode(
    std::size_t node_index, long long px, long long py, double x, double y) {
  const auto& node = nodes_[node_index];
  auto c_pos = node.GetPos(px, py);
  auto c_index = node.c_index[c_pos];

  auto nearest_node_index = (std::size_t)-1;
  auto nearest_point_index = (std::size_t)-1;
  auto nearest_distance2 = std::numeric_limits<double>::max();

  if (c_index == (std::size_t)-1) {
    if (node.IsPoint(c_pos)) {
      nearest_node_index = node_index;
      nearest_point_index = c_index;
      const auto& point = points_[c_index];
      nearest_distance2 = Vec2d(point.x, point.y).DistanceSquareTo(Vec2d(x, y));
    } else {
      std::tie(nearest_node_index, nearest_point_index, nearest_distance2) =
          FindNearestPointInNode(c_index, px, py, x, y);
    }
  }

  for (auto i = 0; i < 4; i++) {
    auto c_index = node.c_index[i];
    if (i != c_pos && c_index != (std::size_t)-1) {
      if (node.IsPoint(c_pos)) {
        const auto& point = points_[c_index];
        auto distance2 = Vec2d(point.x, point.y).DistanceSquareTo(Vec2d(x, y));
        if (distance2 < nearest_distance2) {
          nearest_node_index = node_index;
          nearest_point_index = c_index;
          nearest_distance2 = distance2;
        }
      } else {
        std::size_t r_node_index;
        std::size_t r_point_index;
        double r_distance2;
        std::tie(r_node_index, r_point_index, r_distance2) =
            FindNearestPointOutNode(node_index, px, py, x, y,
                                    nearest_distance2);

        if (r_distance2 < nearest_distance2) {
          nearest_node_index = r_node_index;
          nearest_point_index = r_point_index;
          nearest_distance2 = r_distance;
        }
      }
    }
  }

  return std::make_tuple(nearest_node_index, nearest_point_index,
                         nearest_distance2);
}

std::tuple<std::size_t, std::size_t, double> PCMap::FindNearestPointOutNode(
    std::size_t node_index, long long px, long long py, double x, double y,
    double range) {
  const auto& node = nodes_[node_index];
  auto half_size = HalfSize();
  auto bl = node.cx - half_size;
  auto br = node.cx + half_size;
  auto bt = node.cy + half_size;
  auto bb = node.cy - half_size;

  if (px < bl) {
    if (py < bb) {
    } else if (py >= bb && py < bt) {
    } else {
    }
  } else if (px >= bl && px < br) {
    if (py < bb) {
    } else {
    }
  } else {
    if (py < bb) {
    } else if (py >= bb && py < bt) {
    } else {
    }
  }
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
