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
 * @file lm_matcher.h
 * @brief The class of LMMatcher
 */

#ifndef MODULES_LOCALIZATION_LMD_LM_MATCHER_H_
#define MODULES_LOCALIZATION_LMD_LM_MATCHER_H_

#include <vector>

#include "modules/common/proto/geometry.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/localization/lmd/lm_provider.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class LMMatcher
 *
 * @brief  find the matched lane markers
 */
class LMMatcher {
 public:
  explicit LMMatcher(LMProvider* provider);

  /**
   * @brief  find the matched lane markers
   * @return matched odometry lane markers
   */
  std::vector<OdometryLaneMarker> MatchLaneMarkers(
      const apollo::common::PointENU& position_estimated,
      const apollo::perception::LaneMarkers& lane_markers, double timestamp);

 private:
  LMProvider* provider_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_LM_MATCHER_H_
