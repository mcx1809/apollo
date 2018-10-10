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

#include <iterator>
#include <list>
#include <utility>

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
  explicit PCMapPoint(
      const apollo::localization::OdometryLaneMarkerPoint& point) {
    position = point.position();
    direction = point.direct();
    curvature = point.curvature();
  }
};

class MapNode {
 public:
  /**
   * @brief  Construct map_node according to the given params.
   * @param start_x The x value of upper_left point.
   * @param start_y The y value of upper_left point.
   * @param width   The width value
   * @param height  The height value
   * @param current_level The current map level value
   * @param map_level     The max map level value
   * @param parent_ptr    The pointer to upper_level mapnode
   */
  MapNode(double start_x, double start_y, double width, double height,
          int current_level, int map_level, MapNode* parent_ptr = nullptr);
  ~MapNode();

 public:
  /**
   * @brief  Insert PCMapPoint to MapNode.
   * @param map_point  PCMapPoint to insert.
   */
  void InsertMapPoint(const PCMapPoint& map_point);

  /**
   * @brief  Get the PCMapPoint list of samllest MapNode Range which contains
   * the nearest map points to the given position.
   * @param  position The position to search
   */
  std::list<PCMapPoint> GetMapPoints(const apollo::common::PointENU& position);

 private:
  /**
   * @brief  Remove all PCMapPoint* in MapNode range.
   * @param  map_start_x The x value of upper_left point of MapNode range.
   * @param  map_start_y The y value of upper_left point of MapNode range.
   * @param  map_width   The width value of MapNode range.
   * @param  map_height  The height value of MapNode range.
   */
  void RemoveMapPoints(double map_start_x, double map_start_y, double map_width,
                       double map_height);

  bool IsContain(double map_start_x, double map_start_y, double map_width,
                 double map_height,
                 const apollo::common::PointENU& position) const;

 private:
  double map_start_x_;
  double map_start_y_;
  double map_width_;
  double map_height_;
  int current_map_level_;
  int max_map_level_;
  MapNode* parent;
  MapNode* up_right = nullptr;
  MapNode* up_left = nullptr;
  MapNode* down_left = nullptr;
  MapNode* down_right = nullptr;
  std::list<PCMapPoint> points;
};
/**
 * @class PCMap
 *
 * @brief  Map of point cloud.
 */
class PCMap {
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
   * @return The nearest point in lane_marker or nullptr.
   */
  PCMapPoint* GetNearestPoint(const apollo::common::PointENU& position);

 private:
  LMProvider* provider_;
  MapNode* root = nullptr;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_PC_MAP_H_
