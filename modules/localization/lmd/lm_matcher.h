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
 * @brief The class of LMMatcher.
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
 * @brief  Find the matched lane markers.
 */
class LMMatcher {
 public:
  explicit LMMatcher(LMProvider* provider);
  virtual ~LMMatcher();

  /**
   * @brief  Find the matched lane markers.
   * @param position_estimated The estimated position.
   * @param lane_markers Lane markers from perception.
   * @param timestamp Timestamp for position_estimated and lane_markers.
   * @return The matched odometry lane markers.
   */
  std::vector<OdometryLaneMarker> MatchLaneMarkers(
      const apollo::common::PointENU& position_estimated,
      const apollo::perception::LaneMarkers& lane_markers, double timestamp);

 private:
 /**
   * @brief  Find the best matched position between two lane markers and return the max match score.
   * @param  estimated lane_marker from percption.
   * @param  estimated lane_marker from map.
   * @param the best matched position.
   * @return the max match score.
   */
  double BestMatchScoreForTwoLane(
      const apollo::perception::LaneMarker& per_lane_marker,
      const apollo::localization::OdometryLaneMarker& map_lane_marker,
      double* bestmatch_yposition);

  double GetMold(const std::vector<double>& vec);

  double GetSimilarity(const std::vector<double>& lhs,
                       const std::vector<double>& rhs);

  double Curvature(double a[], double b[], double c[]);

  int Collinear(double a[], double b[], double c[]);

  double Distance(double a[], double b[]);

  LMProvider* provider_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_LM_MATCHER_H_