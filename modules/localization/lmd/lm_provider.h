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
 * @file lm_provider.h
 * @brief The class of LMProvider.
 */

#ifndef MODULES_LOCALIZATION_LMD_LM_PROVIDER_H_
#define MODULES_LOCALIZATION_LMD_LM_PROVIDER_H_

#include <utility>
#include "modules/common/proto/geometry.pb.h"
#include "modules/common/util/file.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/localization/proto/odometry_lane_marker.pb.h"
/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class LMProvider
 *
 * @brief  Provider of  odometry lane markers.
 */
class LMProvider {
 public:
  LMProvider();
  virtual ~LMProvider();
  /**
   * @brief Find the nearest lane marker index with specified position.
   * @param position Specified position.
   */
  void FindNearestLaneMarkerIndex(const apollo::common::PointENU& position);

  /**
   * @brief Get the index of prev lane marker.
   */
  void GetPrevLaneMarkerIndex();

  /**
   * @brief Get the index of next lane marker.
   */
  void GetNextLaneMarkerIndex();

  /**
   * @brief Get the index of left lane marker.
   */
  void GetLeftLaneMarkerIndex();

  /**
   * @brief Get the index of right lane marker.
   */
  void GetRightLaneMarkerIndex();

  /**
   * @brief Get the index of current lane marker.
   */
  const std::pair<int, int>& GetCurrentLaneMarkerIndex() const;
  /**
   * @brief Get lane marker according to index pair.
   * @return A lane marker or nullptr.
   */
  const apollo::localization::OdometryLaneMarker& GetLaneMarker() const;

 private:
  apollo::localization::OdometryLaneMarkersPack LaneMarkersPack_;
  std::pair<int, int> lane_index;
  /**
   * @brief Calclute the distance from point position to the line of start_pos
   * and end_pos.
   * @param position the point
   * @param start_pos the location of start point of the line
   * @param end_pos   the location of end point of the line
   * @return the distance value
   */
  double CalculateDistance(const apollo::common::PointENU& position,
                           const apollo::common::PointENU& start_pos,
                           const apollo::common::PointENU& end_pos) const;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_LM_PROVIDER_H_
