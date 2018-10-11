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

#include "modules/common/log.h"

namespace apollo {
namespace localization {

using apollo::common::Point3D;
using apollo::common::PointENU;
using apollo::common::Status;

MapNode::MapNode(const double start_x, const double start_y, const double width,
                 const double height, const int current_level,
                 const int map_level, const MapNode* parent_ptr)
    : map_start_x_(start_x),
      map_start_y_(start_y),
      map_width_(width),
      map_height_(height),
      current_map_level_(current_level),
      max_map_level_(map_level),
      parent_(parent_ptr) {}

MapNode::~MapNode() {
  RemoveMapPoints(map_start_x_, map_start_y_, map_width_, map_height_);
}

void MapNode::InsertMapPoint(const PCMapPoint& map_point) {
  if (IsContain(map_start_x_, map_start_y_, map_width_, map_height_,
                map_point.position)) {
    if (current_map_level_ == max_map_level_) {
      points_.push_back(map_point);
      return;
    }
    if (IsContain(map_start_x_ + map_width_ / 2, map_start_y_, map_width_ / 2,
                  map_height_ / 2, map_point.position)) {
      if (!up_right_)
        up_right_ = new MapNode(map_start_x_ + map_width_ / 2, map_start_y_,
                               map_width_ / 2, map_height_ / 2,
                               current_map_level_ + 1, max_map_level_, this);
      return up_right_->InsertMapPoint(map_point);
    }
    if (IsContain(map_start_x_, map_start_y_, map_width_ / 2, map_height_ / 2,
                  map_point.position)) {
      if (!up_left_)
        up_left_ = new MapNode(map_start_x_, map_start_y_, map_width_ / 2,
                              map_height_ / 2, current_map_level_ + 1,
                              max_map_level_, this);
      return up_left_->InsertMapPoint(map_point);
    }
    if (IsContain(map_start_x_, map_start_y_ + map_height_ / 2, map_width_ / 2,
                  map_height_ / 2, map_point.position)) {
      if (!down_left_)
        down_left_ = new MapNode(map_start_x_, map_start_y_ + map_height_ / 2,
                                map_width_ / 2, map_height_ / 2,
                                current_map_level_ + 1, max_map_level_, this);
      return down_left_->InsertMapPoint(map_point);
    }
    if (IsContain(map_start_x_ + map_width_ / 2, map_start_y_ + map_height_ / 2,
                  map_width_ / 2, map_height_ / 2, map_point.position)) {
      if (!down_right_)
        down_right_ = new MapNode(map_start_x_ + map_width_ / 2,
                                 map_start_y_ + map_height_ / 2, map_width_ / 2,
                                 map_height_ / 2, current_map_level_ + 1,
                                 max_map_level_, this);
      return down_right_->InsertMapPoint(map_point);
    }
  }
}

void MapNode::RemoveMapPoints(const double map_start_x,
                              const double map_start_y, const double map_width,
                              const double map_height) {
  if (current_map_level_ == max_map_level_) {
    points_.clear();
    return;
  }
  if (up_right_) {
    return up_right_->RemoveMapPoints(
        up_right_->map_start_x_, up_right_->map_start_y_, up_right_->map_width_,
        up_right_->map_height_);
    delete up_right_;
    up_right_= nullptr;
  }
  if (up_left_) {
    return up_left_->RemoveMapPoints(up_left_->map_start_x_,
                                    up_left_->map_start_y_, up_left_->map_width_,
                                    up_left_->map_height_);
    delete up_left_;
    up_left_ = nullptr;
  }
  if (down_left_) {
    return down_left_->RemoveMapPoints(
        down_left_->map_start_x_, down_left_->map_start_y_, down_left_->map_width_,
        down_left_->map_height_);
    delete down_left_;
    down_left_ = nullptr;
  }
  if (down_right_) {
    return down_right_->RemoveMapPoints(
        down_right_->map_start_x_, down_right_->map_start_y_,
        down_right_->map_width_, down_right_->map_height_);
    delete down_right_;
    down_right_ = nullptr;
  }
}

std::list<PCMapPoint> MapNode::GetMapPoints(
    const apollo::common::PointENU& position) const {
  if (up_right_ != nullptr &&
      IsContain(up_right_->map_start_x_, up_right_->map_start_y_,
                up_right_->map_width_, up_right_->map_height_, position)) {
    return up_right_->GetMapPoints(position);
  }
  if (up_left_ != nullptr &&
      IsContain(up_left_->map_start_x_, up_left_->map_start_y_,
                up_left_->map_width_, up_left_->map_height_, position)) {
    return up_left_->GetMapPoints(position);
  }
  if (down_left_ != nullptr &&
      IsContain(down_left_->map_start_x_, down_left_->map_start_y_,
                down_left_->map_width_, down_left_->map_height_, position)) {
    return down_left_->GetMapPoints(position);
  }
  if (down_right_ != nullptr &&
      IsContain(down_right_->map_start_x_, down_right_->map_start_y_,
                down_right_->map_width_, down_right_->map_height_, position)) {
    return down_right_->GetMapPoints(position);
  }
  std::list<PCMapPoint> result;
  if (current_map_level_ == max_map_level_) {
    result.insert(result.begin(), points_.begin(), points_.end());
  }
  return result;
}

bool MapNode::IsContain(const double map_start_x, const double map_start_y,
                        const double map_width, const double map_height,
                        const apollo::common::PointENU& position) const {
  if ((position.x() >= map_start_x) &&
      (position.x() <= (map_start_x + map_width)) &&
      (position.y() >= map_start_y) &&
      (position.y() <= map_start_y + map_width))
    return true;
  return false;
}

PCMap::PCMap(LMProvider* provider) {
  CHECK_NOTNULL(provider);
  provider_ = provider;
}

Status PCMap::UpdateRange(const PointENU& position, const double radius) {
  if (root_ != nullptr) {
    delete root_;
    root_ = nullptr;
  }
  root_ = new MapNode(position.x() - radius / 2, position.y() - radius / 2,
                      radius, radius, 1, 5, nullptr);
  for (int pack_index = 0; pack_index < provider_->GetLaneMarkerPackSize();
       ++pack_index) {
    for (int lane_index = 0;
         lane_index < provider_->GetLaneMarkerSize(pack_index); ++lane_index) {
      auto lane =
          provider_->GetLaneMarker(std::make_pair(pack_index, lane_index));
      if (lane != nullptr) {
        auto lane_points = lane->points();
        for (int point_index = 0; point_index < lane_points.size();
             ++point_index) {
          root_->InsertMapPoint(PCMapPoint(lane_points[point_index]));
        }
      }
    }
  }
  return Status::OK();
}

PCMapPoint* PCMap::GetNearestPoint(const PointENU& position) const {
  std::list<PCMapPoint> point_list = root_->GetMapPoints(position);
  if (point_list.empty()) return nullptr;
  std::list<PCMapPoint>::iterator result_iter = point_list.begin();
  for (std::list<PCMapPoint>::iterator iter = point_list.begin();
       iter != point_list.end(); ++iter) {
    if (provider_->CalculateDistance((*iter).position, position) <
        provider_->CalculateDistance((*result_iter).position, position)) {
      result_iter = iter;
    }
  }
  return &(*result_iter);
}

}  // namespace localization
}  // namespace apollo
