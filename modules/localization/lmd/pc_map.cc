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
namespace {
constexpr int kNeibourNumber = 1;
}
using apollo::common::Point3D;
using apollo::common::PointENU;
using apollo::common::Status;

PCMap::PCMap(LMProvider* provider) {
  CHECK_NOTNULL(provider);
  provider_ = provider;
  auto pack_size = provider_->GetLaneMarkerPackSize();
  for (decltype(pack_size) pack_index = 0; pack_index < pack_size;
       ++pack_index) {
    auto lane_marker_size = provider_->GetLaneMarkerSize(pack_index);
    for (decltype(lane_marker_size) lane_index = 0;
         lane_index < lane_marker_size; ++lane_index) {
      auto lane =
          provider_->GetLaneMarker(std::make_pair(pack_index, lane_index));
      if (lane != nullptr) {
        const auto& lane_points = lane->points();
        auto lane_points_size = lane_points.size();
        for (decltype(lane_points_size) point_index = 0;
             point_index < lane_points_size; ++point_index) {
          const auto& lane_point = lane_points[point_index];
          point_cloud_.push_back(new PCMapPoint(lane_point));
        }
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  auto points_size = point_cloud_.size();
  cloud->points.resize(points_size);
  for (decltype(points_size) point_index = 0; point_index < points_size;
       ++point_index) {
    cloud->points[point_index].x = point_cloud_[point_index]->position.x();
    cloud->points[point_index].y = point_cloud_[point_index]->position.y();
    cloud->points[point_index].z = point_cloud_[point_index]->position.z();
  }
  kd_tree_.setInputCloud(cloud);
}

PCMap::~PCMap() {
  if (point_cloud_.size()) {
    for (auto iter = point_cloud_.begin(); iter != point_cloud_.end(); ++iter) {
      delete *iter;
      *iter = nullptr;
    }
    point_cloud_.clear();
  }
}

Status PCMap::UpdateRange(const PointENU& position, const double radius) {
  return Status::OK();
}

PCMapPoint* PCMap::GetNearestPoint(const PointENU& position) const {
  pcl::PointXYZ search_point;
  search_point.x = position.x();
  search_point.y = position.y();
  search_point.z = position.z();
  std::vector<int> searched_indexes(kNeibourNumber);
  std::vector<float> squared_distances(kNeibourNumber);
  if (kd_tree_.nearestKSearch(search_point, kNeibourNumber, searched_indexes,
                              squared_distances) > 0) {
    ADEBUG
        << " the point in point_cloud_ list with index " << searched_indexes[0]
        << " is the nearest point to required position with distance equal to "
        << squared_distances[0] << std::endl;
    return point_cloud_[searched_indexes[0]];
  }
  return nullptr;
}

}  // namespace localization
}  // namespace apollo
