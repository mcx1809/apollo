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

#include "modules/common/proto/geometry.pb.h"
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
  /**
   * @brief Find the nearest lane marker  with specified position.
   * @param position Specified position.
   * @return A lane marker or nullptr.
   */
  const apollo::localization::OdometryLaneMarker* FindNearestLaneMarker(
      const apollo::common::PointENU& position) const;

  /**
   * @brief Get the prev lane marker.
   * @param lane_marker Current lane marker.
   * @return A lane marker or nullptr.
   */
  const apollo::localization::OdometryLaneMarker* GetPrevLaneMarker(
      const apollo::localization::OdometryLaneMarker& lane_marker) const;

  /**
   * @brief Get the next lane marker.
   * @param lane_marker Current lane marker.
   * @rreturn A lane marker or nullptr.
   */
  const apollo::localization::OdometryLaneMarker* GetNextLaneMarker(
      const apollo::localization::OdometryLaneMarker& lane_marker) const;

  /**
   * @brief Get the left lane marker.
   * @param lane_marker Current lane marker.
   * @return A lane marker or nullptr.
   */
  const apollo::localization::OdometryLaneMarker* GetLeftLaneMarker(
      const apollo::localization::OdometryLaneMarker& lane_marker) const;

  /**
   * @brief Get the right lane marker.
   * @param lane_marker Current lane marker.
   * @return A lane marker or nullptr.
   */
  const apollo::localization::OdometryLaneMarker* GetRightLaneMarker(
      const apollo::localization::OdometryLaneMarker& lane_marker) const;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_LM_PROVIDER_H_
