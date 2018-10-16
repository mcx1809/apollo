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

/**
 * @file pc_map.h
 * @brief The class of PCMap.
 */

#ifndef MODULES_LOCALIZATION_LMD_PC_MAP_H_
#define MODULES_LOCALIZATION_LMD_PC_MAP_H_

#include <map>

#include "modules/common/proto/geometry.pb.h"

#include "modules/common/status/status.h"
#include "modules/localization/lmd/lm_provider.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @struct PCMapPoint
 * @brief  Point stored in map.
 */
struct PCMapPoint {
  PCMapPoint* prev = nullptr;
  PCMapPoint* next = nullptr;
  apollo::common::PointENU position;
  apollo::common::Point3D direction;
  double curvature;

  explicit PCMapPoint(const OdometryLaneMarkerPoint& point) {
    position = point.position();
    direction = point.direct();
    curvature = point.curvature();
  }
};

/**
 * @class PCMap
 *
 * @brief  Map of point cloud.
 */
class PCMap {
  struct Index2D {
    int64_t x;
    int64_t y;

    bool operator<(const Index2D& other) const {
      if (x < other.x)
        return true;
      else if (x == other.x)
        return y < other.y;
      else
        return false;
    }
  };

  struct Node {
    std::map<Index2D, PCMapPoint> points;
  };

 public:
  explicit PCMap(LMProvider* provider);

  /**
   * @brief  Update map for range.
   * @param position The position of center point.
   * @param the radius.
   * @return Status::OK() if a suitable speed-data is created; error otherwise.
   */
  apollo::common::Status UpdateRange(const apollo::common::PointENU& position,
                                     double radius);

  /**
   * @brief  Find the nearest point in lane_marker according to the given
   * position.
   * @param position The given position.
   * @param d2 Distance squqre.
   * @return The nearest point in lane_marker or nullptr.
   */
  const PCMapPoint* GetNearestPoint(const apollo::common::PointENU& position,
                                    double* d2) const;

  /**
   * @brief  Find the nearest point in lane_marker according to the given
   * position.
   * @param position The given position.
   * @return The nearest point in lane_marker or nullptr.
   */
  const PCMapPoint* GetNearestPoint(
      const apollo::common::PointENU& position) const;

 private:
  void LoadLaneMarker(const OdometryLaneMarker& lane_marker);
  Index2D MakeNodeIndex(double x, double y) const;

 private:
  LMProvider* provider_;
  std::map<Index2D, Node> nodes_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_PC_MAP_H_
