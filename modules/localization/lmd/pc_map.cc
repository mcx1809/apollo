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
  // initialize root node
  nodes_.resize(1);
  auto& root = nodes_[0];
  root.level = kMapSizeMaxLevel;
  root.cx = 0;
  root.cy = 0;

  // load all of data from provider
  if (provider != nullptr) {
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
}

Status PCMap::UpdateRange(const PointENU& position, double radius) {
  return Status::OK();
}

const PCMapIndex PCMap::GetNearestPoint(const PointENU& position,
                                        double* d2) const {
  auto x = position.x();
  auto y = position.y();
  auto px = GetMapX(x);
  auto py = GetMapX(y);
  auto root_index = 0;
  if (!nodes_[root_index].OnBoundary(px, py)) return (PCMapIndex)-1;

  PCMapIndex point_index;
  double distance2;
  std::tie(std::ignore, point_index, distance2, std::ignore) =
      FindNearestPointInNode(root_index, px, py, x, y);

  if (d2 != nullptr) *d2 = distance2;
  return point_index;
}

const PCMapIndex PCMap::GetNearestPoint(const PointENU& position) const {
  return GetNearestPoint(position, nullptr);
}

const PCMapPoint* PCMap::Point(PCMapIndex index) const {
  if (index == (PCMapIndex)-1) return nullptr;
  return &points_[index];
}

void PCMap::LoadLaneMarker(const OdometryLaneMarker& lane_marker) {
  auto seg_point_index = (PCMapIndex)-1;
  PCMapIndex last_node_index = 0;
  for (const auto& lane_point : lane_marker.points()) {
    auto point_index = FetchPoint();
    auto& point = points_[point_index];
    point.Set(lane_point);

    last_node_index = InsertPoint(last_node_index, point_index);
    if (last_node_index == (PCMapIndex)-1) {
      StorePoint(point_index);
      seg_point_index = (PCMapIndex)-1;
      last_node_index = 0;
      continue;
    }

    if (seg_point_index == (PCMapIndex)-1) {
      if (point.next == (PCMapIndex)-1) seg_point_index = point_index;
    } else {
      if (point.prev == (PCMapIndex)-1) {
        point.prev = seg_point_index;
        auto& seg_point = points_[seg_point_index];
        seg_point.next = point_index;
      }
      if (point.next == (PCMapIndex)-1)
        seg_point_index = point_index;
      else
        seg_point_index = (PCMapIndex)-1;
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

PCMapIndex PCMap::InsertPoint(PCMapIndex node_index, PCMapIndex point_index) {
  const auto& point = points_[point_index];
  auto px = GetMapX(point.position.x());
  auto py = GetMapY(point.position.y());
  return InsertPoint(node_index, point_index, px, py);
}

PCMapIndex PCMap::InsertPoint(PCMapIndex node_index, PCMapIndex point_index,
                              long long px, long long py) {
  const auto& node = nodes_[node_index];
  if (node.OnBoundary(px, py)) {
    return InsertPointInNode(node_index, point_index, px, py);
  } else {
    if (node.p_index != (PCMapIndex)-1)
      return InsertPoint(node.p_index, point_index, px, py);
    else
      return (PCMapIndex)-1;
  }
}

PCMapIndex PCMap::InsertPointInNode(PCMapIndex node_index,
                                    PCMapIndex point_index, long long px,
                                    long long py) {
  auto& node = nodes_[node_index];
  auto c_pos = node.GetPos(px, py);
  auto c_index = node.c_index[c_pos];
  if (c_index == (PCMapIndex)-1) {
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
        //
        std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
        return m_index;
      }
      m_pos = p_pos;
    }

    auto& point = points_[point_index];
    point.prev = cur_point.prev;
    point.next = cur_point.next;
    if (point.prev != (PCMapIndex)-1) {
      auto& prev_point = points_[point.prev];
      prev_point.next = point_index;
    }
    if (point.next != (PCMapIndex)-1) {
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
    //
    std::cout << "<<<<<<<<<<<<<<<<<<<< " << node_index << " " << m_pos
              << std::endl;
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

std::tuple<PCMapIndex, PCMapIndex, double, bool> PCMap::FindNearestPointInNode(
    PCMapIndex node_index, long long px, long long py, double x,
    double y) const {
  const auto& node = nodes_[node_index];
  auto c_pos = node.GetPos(px, py);
  auto c_index = node.c_index[c_pos];

  auto nearest_node_index = (PCMapIndex)-1;
  auto nearest_point_index = (PCMapIndex)-1;
  auto nearest_distance2 = std::numeric_limits<double>::max();
  bool finished = false;

  //
  std::cout << "===------------------------ " << node_index << " " << c_pos
            << std::endl;
  if (c_index != (PCMapIndex)-1) {
    //
    std::cout << "1==------------------------ " << c_index << std::endl;
    if (node.IsPoint(c_pos)) {
      //
      std::cout << "------------+++++ " << std::endl;
      nearest_node_index = node_index;
      nearest_point_index = c_index;
      const auto& point = points_[c_index];
      nearest_distance2 = Vec2d(point.position.x(), point.position.y())
                              .DistanceSquareTo(Vec2d(x, y));
    } else {
      const auto& c_node = nodes_[c_index];
      //
      std::cout << "------------ " << std::endl;
      if (c_node.OnBoundary(px, py))
        std::tie(nearest_node_index, nearest_point_index, nearest_distance2,
                 finished) = FindNearestPointInNode(c_index, px, py, x, y);
      else
        std::tie(nearest_node_index, nearest_point_index, nearest_distance2) =
            FindNearestPointOutNode(c_index, px, py, x, y, nearest_distance2);
    }
  }

  if (!finished) {
    for (auto i = 0; i < 4; ++i) {
      auto c_index = node.c_index[i];
      if (i != c_pos && c_index != (PCMapIndex)-1) {
        if (node.IsPoint(i)) {
          const auto& point = points_[c_index];
          auto distance2 = Vec2d(point.position.x(), point.position.y())
                               .DistanceSquareTo(Vec2d(x, y));
          if (distance2 < nearest_distance2) {
            nearest_node_index = node_index;
            nearest_point_index = c_index;
            nearest_distance2 = distance2;
          }
        } else {
          PCMapIndex r_node_index;
          PCMapIndex r_point_index;
          double distance2;
          std::tie(r_node_index, r_point_index, distance2) =
              FindNearestPointOutNode(node_index, px, py, x, y,
                                      nearest_distance2);

          if (distance2 < nearest_distance2) {
            nearest_node_index = r_node_index;
            nearest_point_index = r_point_index;
            nearest_distance2 = distance2;
          }
        }
      }
    }

    auto half_size = node.HalfSize();
    auto l = x - (node.cx - half_size) * kMapResolution;
    auto r = (node.cx + half_size) * kMapResolution - x;
    auto t = (node.cy + half_size) * kMapResolution - y;
    auto b = y - (node.cy - half_size) * kMapResolution;
    finished = (l * l > nearest_distance2) && (r * r > nearest_distance2) &&
               (t * t > nearest_distance2) && (b * b > nearest_distance2);
  }

  return std::make_tuple(nearest_node_index, nearest_point_index,
                         nearest_distance2, finished);
}

std::tuple<PCMapIndex, PCMapIndex, double> PCMap::FindNearestPointOutNode(
    PCMapIndex node_index, long long px, long long py, double x, double y,
    double range2) const {
  const auto& node = nodes_[node_index];
  auto half_size = node.HalfSize();
  auto bl = node.cx - half_size;
  auto br = node.cx + half_size;
  auto bt = node.cy + half_size;
  auto bb = node.cy - half_size;

  auto get_distance2 = [&](long long xi, long long yi) {
    auto xd = xi * kMapResolution;
    auto yd = yi * kMapResolution;
    return Vec2d(xd, yd).DistanceSquareTo(Vec2d(x, y));
  };

  bool out;
  if (px < bl) {
    if (py < bb) {
      out = get_distance2(bl, bb) > range2;
    } else if (py >= bb && py < bt) {
      auto d = (bl - px) * kMapResolution;
      out = d * d > range2;
    } else {
      out = get_distance2(bl, bt) > range2;
    }
  } else if (px >= bl && px < br) {
    if (py < bb) {
      auto d = (bb - py) * kMapResolution;
      out = d * d > range2;
    } else {
      auto d = (py - bt) * kMapResolution;
      out = d * d > range2;
    }
  } else {
    if (py < bb) {
      out = get_distance2(br, bb) > range2;
    } else if (py >= bb && py < bt) {
      auto d = (px - br) * kMapResolution;
      out = d * d > range2;
    } else {
      out = get_distance2(br, bt) > range2;
    }
  }
  if (out)
    return std::make_tuple((PCMapIndex)-1, (PCMapIndex)-1,
                           std::numeric_limits<double>::max());

  auto nearest_node_index = (PCMapIndex)-1;
  auto nearest_point_index = (PCMapIndex)-1;
  auto nearest_distance2 = std::numeric_limits<double>::max();

  for (auto i = 0; i < 4; ++i) {
    auto c_index = node.c_index[i];
    if (c_index != (PCMapIndex)-1) {
      if (node.IsPoint(i)) {
        const auto& point = points_[c_index];
        auto distance2 = Vec2d(point.position.x(), point.position.y())
                             .DistanceSquareTo(Vec2d(x, y));
        if (distance2 < nearest_distance2 && distance2 < range2) {
          nearest_node_index = node_index;
          nearest_point_index = c_index;
          nearest_distance2 = distance2;
        }
      } else {
        PCMapIndex r_node_index;
        PCMapIndex r_point_index;
        double distance2;
        std::tie(r_node_index, r_point_index, distance2) =
            FindNearestPointOutNode(node_index, px, py, x, y,
                                    nearest_distance2);

        if (distance2 < nearest_distance2) {
          nearest_node_index = r_node_index;
          nearest_point_index = r_point_index;
          nearest_distance2 = distance2;
        }
      }
    }
  }

  return std::make_tuple(nearest_node_index, nearest_point_index,
                         nearest_distance2);
}

long long PCMap::GetMapX(double x) const {
  return (long long)(x / kMapResolution);
}

long long PCMap::GetMapY(double y) const {
  return (long long)(y / kMapResolution);
}

PCMapIndex PCMap::FetchPoint() {
  if (free_point_head_ != (PCMapIndex)-1) {
    auto index = free_point_head_;
    auto& point = points_[index];
    free_point_head_ = point.next;
    point.next = (PCMapIndex)-1;
    return index;
  } else {
    points_.resize(points_.size() + 1);
    return points_.size() - 1;
  }
}

void PCMap::StorePoint(PCMapIndex index) {
  auto& point = points_[index];
  point.prev = (PCMapIndex)-1;
  point.next = free_point_head_;
  free_point_head_ = index;
}

PCMapIndex PCMap::FetchNode() {
  if (free_node_head_ != (PCMapIndex)-1) {
    auto index = free_node_head_;
    auto& node = nodes_[index];
    free_node_head_ = node.next;
    node.next = (PCMapIndex)-1;
    return index;
  } else {
    nodes_.resize(nodes_.size() + 1);
    return nodes_.size() - 1;
  }
}

void PCMap::StoreNode(PCMapIndex index) {
  auto& node = nodes_[index];
  node.next = free_node_head_;
  free_node_head_ = index;
}

}  // namespace localization
}  // namespace apollo
