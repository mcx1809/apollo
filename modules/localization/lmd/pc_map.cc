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

MapNode::MapNode(double start_x, double start_y, double width, double height,
                 int current_level, int map_level, MapNode* parent_ptr)
    : map_start_x_(start_x),
      map_start_y_(start_y),
      map_width_(width),
      map_height_(height),
      current_map_level_(current_level),
      max_map_level_(map_level),
      parent(parent_ptr) {}

MapNode::~MapNode() {
  RemoveMapPoints(map_start_x_, map_start_y_, map_width_, map_height_);
}

void MapNode::InsertMapPoint(const PCMapPoint& map_point) {
  if (IsContain(map_start_x_, map_start_y_, map_width_, map_height_,
                map_point.position)) {
    if (current_map_level_ == max_map_level_) {
      points.push_back(map_point);
      return;
    }
    if (IsContain(map_start_x_ + map_width_ / 2, map_start_y_, map_width_ / 2,
                  map_height_ / 2, map_point.position)) {
      if (!up_right)
        up_right = new MapNode(map_start_x_ + map_width_ / 2, map_start_y_,
                               map_width_ / 2, map_height_ / 2,
                               current_map_level_ + 1, max_map_level_, this);
      up_right->InsertMapPoint(map_point);
    } else if (IsContain(map_start_x_, map_start_y_, map_width_ / 2,
                         map_height_ / 2, map_point.position)) {
      if (!up_left)
        up_left = new MapNode(map_start_x_, map_start_y_, map_width_ / 2,
                              map_height_ / 2, current_map_level_ + 1,
                              max_map_level_, this);
      up_left->InsertMapPoint(map_point);
    } else if (IsContain(map_start_x_, map_start_y_ + map_height_ / 2,
                         map_width_ / 2, map_height_ / 2, map_point.position)) {
      if (!down_left)
        down_left = new MapNode(map_start_x_, map_start_y_ + map_height_ / 2,
                                map_width_ / 2, map_height_ / 2,
                                current_map_level_ + 1, max_map_level_, this);
      down_left->InsertMapPoint(map_point);
    } else if (IsContain(map_start_x_ + map_width_ / 2,
                         map_start_y_ + map_height_ / 2, map_width_ / 2,
                         map_height_ / 2, map_point.position)) {
      if (!down_right)
        down_right = new MapNode(map_start_x_ + map_width_ / 2,
                                 map_start_y_ + map_height_ / 2, map_width_ / 2,
                                 map_height_ / 2, current_map_level_ + 1,
                                 max_map_level_, this);
      down_right->InsertMapPoint(map_point);
    }
  }
}

void MapNode::RemoveMapPoints(double map_start_x, double map_start_y,
                              double map_width, double map_height) {
  if (current_map_level_ == max_map_level_) {
    points.clear();
    return;
  }
  if (up_right) {
    up_right->RemoveMapPoints(map_start_x_ + map_width_ / 2, map_start_y_,
                              map_width_ / 2, map_height_ / 2);
    delete up_right;
    up_right = nullptr;
  }
  if (up_left) {
    up_left->RemoveMapPoints(map_start_x_, map_start_y_, map_width_ / 2,
                             map_height_ / 2);
    delete up_left;
    up_left = nullptr;
  }
  if (down_left) {
    down_left->RemoveMapPoints(map_start_x_, map_start_y_ + map_height_ / 2,
                               map_width_ / 2, map_height_ / 2);
    delete down_left;
    down_left = nullptr;
  }
  if (down_right) {
    down_right->RemoveMapPoints(map_start_x_ + map_width_ / 2,
                                map_start_y_ + map_height_ / 2, map_width_ / 2,
                                map_height_ / 2);
    delete down_right;
    down_right = nullptr;
  }
}

std::list<PCMapPoint> MapNode::GetMapPoints(
    const apollo::common::PointENU& position) {
  std::cout << "VVVVVVVVVVVVVVVVVVVVVV" << map_start_x_ << " " << map_start_y_
            << " " << map_height_ << " " << map_height_ << " " << position.x()
            << " " << position.y() << " " << current_map_level_ << " "
            << max_map_level_ << std::endl;
  std::cout << IsContain(map_start_x_ + map_width_ / 2, map_start_y_,
                         map_width_ / 2, map_height_ / 2, position)
            << " Contains in up_right " << std::endl;
  std::cout << IsContain(map_start_x_, map_start_y_, map_width_ / 2,
                         map_height_ / 2, position)
            << " Contains in up_left" << std::endl;
  std::cout << IsContain(map_start_x_, map_start_y_ + map_height_ / 2,
                         map_width_ / 2, map_height_ / 2, position)
            << " Contains in down_left" << std::endl;
  std::cout << IsContain(map_start_x_ + map_width_ / 2,
                         map_start_y_ + map_height_ / 2, map_width_ / 2,
                         map_height_ / 2, position)
            << " Contains in down_right" << std::endl;
  if (current_map_level_ == max_map_level_) {
    std::cout << points.size() << " points in result list" << std::endl;
    return points;
  }
  if (up_right != nullptr &&
      IsContain(map_start_x_ + map_width_ / 2, map_start_y_, map_width_ / 2,
                map_height_ / 2, position)) {
    up_right->GetMapPoints(position);
  }
  if (up_left != nullptr &&
      IsContain(map_start_x_, map_start_y_, map_width_ / 2, map_height_ / 2,
                position)) {
    up_left->GetMapPoints(position);
  }

  if (down_left != nullptr &&
      IsContain(map_start_x_, map_start_y_ + map_height_ / 2, map_width_ / 2,
                map_height_ / 2, position)) {
    down_left->GetMapPoints(position);
  }
  if (down_right != nullptr &&
      IsContain(map_start_x_ + map_width_ / 2, map_start_y_ + map_height_ / 2,
                map_width_ / 2, map_height_ / 2, position)) {
    down_right->GetMapPoints(position);
  }
}

bool MapNode::IsContain(double map_start_x, double map_start_y,
                        double map_width, double map_height,
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

Status PCMap::UpdateRange(const PointENU& position, double radius) {
  if (root != nullptr) {
    delete root;
    root = nullptr;
  }
  root = new MapNode(position.x() - radius / 2, position.y() - radius / 2,
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
          root->InsertMapPoint(PCMapPoint(lane_points[point_index]));
        }
      }
    }
  }
  return Status::OK();
}

PCMapPoint* PCMap::GetNearestPoint(const PointENU& position) {
  std::list<PCMapPoint> point_list = root->GetMapPoints(position);
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
