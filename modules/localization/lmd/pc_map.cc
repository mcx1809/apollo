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

void MapNode::InsertMapPoint(PCMapPoint* object) {
  if (current_map_level_ == max_map_level_) {
    points.push_back(object);
    return;
  }
  if (IsContain(map_start_x_ + map_width_ / 2, map_start_y_, map_width_ / 2,
                map_height_ / 2, object->position.x(), object->position.y())) {
    if (!up_right)
      up_right = new MapNode(map_start_x_ + map_width_ / 2, map_start_y_,
                             map_width_ / 2, map_height_ / 2,
                             current_map_level_ + 1, max_map_level_, this);
    up_right->InsertMapPoint(object);
    return;
  } else if (IsContain(map_start_x_, map_start_y_, map_width_ / 2,
                       map_height_ / 2, object->position.x(),
                       object->position.y())) {
    if (!up_left)
      up_left = new MapNode(map_start_x_, map_start_y_, map_width_ / 2,
                            map_height_ / 2, current_map_level_ + 1,
                            max_map_level_, this);
    up_left->InsertMapPoint(object);
    return;
  } else if (IsContain(map_start_x_, map_start_y_ + map_height_ / 2,
                       map_width_ / 2, map_height_ / 2, object->position.x(),
                       object->position.y())) {
    if (!down_left)
      down_left = new MapNode(map_start_x_, map_start_y_ + map_height_ / 2,
                              map_width_ / 2, map_height_ / 2,
                              current_map_level_ + 1, max_map_level_, this);
    down_left->InsertMapPoint(object);
    return;
  } else if (IsContain(map_start_x_ + map_width_ / 2,
                       map_start_y_ + map_height_ / 2, map_width_ / 2,
                       map_height_ / 2, object->position.x(),
                       object->position.y())) {
    if (!down_right)
      down_right = new MapNode(map_start_x_ + map_width_ / 2,
                               map_start_y_ + map_height_ / 2, map_width_ / 2,
                               map_height_ / 2, current_map_level_ + 1,
                               max_map_level_, this);
    down_right->InsertMapPoint(object);
    return;
  }
}

void MapNode::RemoveMapPoints(float map_start_x, float map_start_y,
                              float map_width, float map_height) {
  if (IsContain(map_start_x, map_start_y, map_width, map_height, this)) {
    if (current_map_level_ == max_map_level_) {
      points.clear();
      return;
    }
  }
  if (up_right && IsContain(map_start_x_ + map_width_ / 2, map_start_y_,
                            map_width_ / 2, map_height_ / 2, up_right)) {
    up_right->RemoveMapPoints(map_start_x_ + map_width_ / 2, map_start_y_,
                              map_width_ / 2, map_height_ / 2);
    delete up_right;
    up_right = nullptr;
  }
  if (up_left && IsContain(map_start_x_, map_start_y_, map_width_ / 2,
                           map_height_ / 2, up_left)) {
    up_left->RemoveMapPoints(map_start_x_, map_start_y_, map_width_ / 2,
                             map_height_ / 2);
    delete up_left;
    up_left = nullptr;
  }
  if (down_left && IsContain(map_start_x_, map_start_y_ + map_height_ / 2,
                             map_width_ / 2, map_height_ / 2, down_left)) {
    down_left->RemoveMapPoints(map_start_x_, map_start_y_ + map_height_ / 2,
                               map_width_ / 2, map_height_ / 2);
    delete down_left;
    down_left = nullptr;
  }
  if (down_right &&
      IsContain(map_start_x_ + map_width_ / 2, map_start_y_ + map_height_ / 2,
                map_width_ / 2, map_height_ / 2, down_right)) {
    down_right->RemoveMapPoints(map_start_x_ + map_width_ / 2,
                                map_start_y_ + map_height_ / 2, map_width_ / 2,
                                map_height_ / 2);
    delete down_right;
    down_right = nullptr;
  }
}

std::list<PCMapPoint*> MapNode::GetMapPoints(float pos_x, float pos_y) {
  std::list<PCMapPoint*> result;
  if (current_map_level_ == max_map_level_) {
    result.insert(result.begin(), points.begin(), points.end());
    return result;
  }
  if (up_right && IsContain(map_start_x_ + map_width_ / 2, map_start_y_,
                            map_width_ / 2, map_height_ / 2, pos_x, pos_y)) {
    return up_right->GetMapPoints(pos_x, pos_y);
  }
  if (up_left && IsContain(map_start_x_, map_start_y_, map_width_ / 2,
                           map_height_ / 2, pos_x, pos_y)) {
    return up_left->GetMapPoints(pos_x, pos_y);
  }
  if (down_left && IsContain(map_start_x_, map_start_y_ + map_height_ / 2,
                             map_width_ / 2, map_height_ / 2, pos_x, pos_y)) {
    return down_left->GetMapPoints(pos_x, pos_y);
  }
  if (down_right &&
      IsContain(map_start_x_ + map_width_ / 2, map_start_y_ + map_height_ / 2,
                map_width_ / 2, map_height_ / 2, pos_x, pos_y)) {
    return down_right->GetMapPoints(pos_x, pos_y);
  }
  return result;
}

bool MapNode::IsContain(float map_start_x, float map_start_y, float map_width,
                        float map_height, MapNode* sub_node) const {
  if ((sub_node->map_start_x_ >= map_start_x) &&
      (sub_node->map_start_x_ + sub_node->map_width_ <=
       (map_start_x + map_width)) &&
      (sub_node->map_start_y_ >= map_start_y) &&
      (sub_node->map_start_x_ + sub_node->map_height_ <=
       map_start_y + map_width))
    return true;
  return false;
}

bool MapNode::IsContain(float map_start_x, float map_start_y, float map_width,
                        float map_height, float pos_x, float pos_y) const {
  if ((pos_x >= map_start_x) && (pos_x <= (map_start_x + map_width)) &&
      (pos_y >= map_start_y) && (pos_y <= map_start_y + map_width))
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
          root->InsertMapPoint(new PCMapPoint(lane_points[point_index]));
        }
      }
    }
  }
  return Status::OK();
}

PCMapPoint* PCMap::GetNearestPoint(const PointENU& position) {
  std::list<PCMapPoint*> point_list =
      root->GetMapPoints(position.x(), position.y());
  if (point_list.empty()) return nullptr;
  std::list<PCMapPoint*>::iterator result_iter = point_list.begin();
  for (std::list<PCMapPoint*>::iterator iter = point_list.begin();
       iter != point_list.end(); ++iter) {
    if (provider_->CalculateDistance((*iter)->position, position) <
        provider_->CalculateDistance((*result_iter)->position, position)) {
      result_iter = iter;
    }
  }
  return *result_iter;
}

}  // namespace localization
}  // namespace apollo
