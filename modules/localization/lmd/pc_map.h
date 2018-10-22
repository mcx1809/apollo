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
#include <vector>

#include "modules/common/math/math_utils.h"
#include "modules/common/proto/geometry.pb.h"
#include "modules/common/status/status.h"

#include "modules/localization/lmd/lm_provider.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

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

  /**
   * @brief insert the points in the given OdometryLaneMarker to nodes.
   * @param lane_marker The lane_marker to sample points
   */
  void LoadLaneMarker(const OdometryLaneMarker& lane_marker);

  /**
   * @brief  Prepare odometry lane markers according to the source perception
   * lane_markers and given params
   * @param source The source perception lane markers.
   * @param position The ENU position after pc registration
   * @param heading The heading value after pc registration
   * @param lane_length The length to sample points
   * @param point_number The total sample point number
   * @return The vector contains the generated odometry lane markers
   */
  std::vector<OdometryLaneMarker> PrepareLaneMarkers(
      const apollo::perception::LaneMarkers& source,
      const apollo::common::PointENU position, const double heading,
      const double lane_length, const int point_number);

 private:
  /**
   *@brief Maker node index data struct according to given params
   *@param x The x value to make node index
   *@param y The y value to make node index
   *@return The generated Index2D data struct by given params
   */
  Index2D MakeNodeIndex(const double x, const double y) const;

  /**
   * @brief  Generate odometry lane marker according to the source perception
   * lane_marker and given params
   * @param lanemarker The source perception lanemarker.
   * @param position The ENU position after pc registration
   * @param heading The heading value after pc registration
   * @param lane_length The length to sample points
   * @param point_number The total sample point number
   * @return The generated odometry lane marker
   */
  const OdometryLaneMarker GenerateOdometryLaneMarker(
      const apollo::perception::LaneMarker& lanemarker,
      const apollo::common::PointENU position, const double heading,
      const double lane_length, const int point_number) const;

  /**
   * @brief  Calculate curve value by given curve params.
   * @param  x_value: value of x.
   *         c0: position.
   *         c1: heading_angle.
   *         c2: curvature.
   *         c3: curvature_derivative.
   * @return y = c3 * x**3 + c2 * x**2 + c1 * x + c0.
   */
  double CalCurveValue(const double x_value, const double c0, const double c1,
                       const double c2, const double c3) const;

  /**
   * @brief  Calculate the first derivative value according to x_value and curve
   * analysis formula y = c3 * x**3 + c2 * x**2 + c1 * x + c0
   * @param  x_value: value of x.
   *         c0: position.
   *         c1: heading_angle.
   *         c2: curvature.
   *         c3: curvature_derivative.
   * @return the first derivative value when x equal to x_value
   */
  double CalDerivative(const double x_value, const double c0, const double c1,
                       const double c2, const double c3) const;

  /**
    * @brief  Calculate the curvity value according to x_value and curve
    analysis formula y = c3 * x**3 + c2 * x**2 + c1 * x + c0
    * @param  x_value: value of x.
    *         c0: position.
    *         c1: heading_angle.
    *         c2: curvature.
    *         c3: curvature_derivative.
    * @return K = |y''| / (1 + y'**2)**(3.0/2)
            curvity_value K according to the analysis formula with x = x_value
    */
  double CalCurvity(const double x_value, const double c0, const double c1,
                    const double c2, const double c3) const;

 private:
  LMProvider* provider_;
  std::map<Index2D, Node> nodes_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_PC_MAP_H_
