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
 * @file lm_bag_processor.h
 * @brief The class of LMBagProcessor.
 */
#ifndef MODULES_LOCALIZATION_LMD_LM_BAG_PROCESSOR_H_
#define MODULES_LOCALIZATION_LMD_LM_BAG_PROCESSOR_H_

#include <string>
#include <utility>
#include <vector>

#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "modules/common/math/math_utils.h"
#include "modules/common/proto/geometry.pb.h"
#include "modules/common/util/file.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/odometry_lane_marker.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/common/adapters/adapter_gflags.h"

#include "modules/localization/lmd/lm_sampler.h"
/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */

namespace apollo {
namespace localization {
class LMProcessor {
 public:
  explicit LMProcessor(const std::string filename);

  bool IsDoubleEqual(const double first_value, const double second_value) const;

  bool InterpolatePose(const double timestamp_sec,
                       apollo::common::PointENU* location_ptr,
                       double* heading) const;

  void PrepareMarkersPack(apollo::localization::OdometryLaneMarkersPack*
                              lane_markers_pack_ptr) const;

  void SerializeToFile(const apollo::localization::OdometryLaneMarkersPack*
                           lane_markers_pack_ptr) const;

  double GetCurveVal(const double x_value, const double c0, const double c1,
                     const double c2, const double c3) const;

  double GetDerivative(const double x_value, const double c0, const double c1,
                       const double c2, const double c3) const;

  double GetCurvity(const double x_value, const double c0, const double c1,
                    const double c2, const double c3) const;

 private:
  std::vector<apollo::localization::LocalizationEstimate> localization_msgs_;
  std::vector<apollo::perception::PerceptionObstacles> obstacle_msgs_;
};
}  // namespace localization
}  // namespace apollo
#endif  // MODULES_LOCALIZATION_LMD_LM_BAG_PROCESSOR_H_
