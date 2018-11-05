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
  /**
   * @brief  prepare desired markers pack in given bag file and store to the
   * lane_markers_pack
   * @param  lane_markers_pack_ptr: the ptr to the store OdometryLaneMarkersPack
   * info
   */
  void PrepareMarkersPack(apollo::localization::OdometryLaneMarkersPack*
                              lane_markers_pack_ptr) const;

 private:
  std::vector<apollo::localization::LocalizationEstimate> localization_msgs_;
  std::vector<apollo::perception::PerceptionObstacles> obstacle_msgs_;
  /**
   * @brief  Interpolate the location and heading value according to the given
   * timestamp
   * @param  timestamp_sec: the given timestamp to search location and heading
   * value
   * @param  location_ptr: ptr to the location
   * @param  heading_ptr: ptr to heading
   * @return true if succeed
   */
  bool InterpolatePose(const double timestamp_sec,
                       apollo::common::PointENU* location_ptr,
                       double* heading_ptr) const;
  /**
   * @brief  Serialize the given OdometryLaneMarkersPack to desired bin file
   * @param  lane_markers_pack_ptr: the ptr to the given OdometryLaneMarkersPack
   */
  void SerializeToFile(const apollo::localization::OdometryLaneMarkersPack*
                           lane_markers_pack_ptr) const;
  /**
   * @brief  Calculate curve value by given curve params.
   * @param  x_value: value of x.
   * @param  c0: position.
   * @param  c1: heading_angle.
   * @param  c2: curvature.
   * @param  c3: curvature_derivative.
   * @return y = c3 * x**3 + c2 * x**2 + c1 * x + c0.
   */
  double GetCurveVal(const double x_value, const double c0, const double c1,
                     const double c2, const double c3) const;
  /**
   * @brief  Calculate the first derivative value according to x_value and curve
   * analysis formula y = c3 * x**3 + c2 * x**2 + c1 * x + c0
   * @param  x_value: value of x.
   * @param  c0: position.
   * @param  c1: heading_angle.
   * @param  c2: curvature.
   * @param  c3: curvature_derivative.
   * @return the first derivative value when x equal to x_value
   */
  double GetDerivative(const double x_value, const double c0, const double c1,
                       const double c2, const double c3) const;
  /**
    * @brief  Calculate the curvity value according to x_value and curve
    analysis formula y = c3 * x**3 + c2 * x**2 + c1 * x + c0
    * @param  x_value: value of x.
    * @param  c0: position.
    * @param  c1: heading_angle.
    * @param  c2: curvature.
    * @param  c3: curvature_derivative.
    * @return K = |y''| / (1 + y'**2)**(3.0/2)
              curvity_value K according to the analysis formula with x = x_value
    */
  double GetCurvity(const double x_value, const double c0, const double c1,
                    const double c2, const double c3) const;
  /**
   * @brief  Judge two timestamp is equal in a given range.
   * @param  first_value: first timestamp to compare
   * @param  second_value: second timestamp to compare.
   * @return true if the offset in a given value
   */
  bool IsDoubleEqual(const double first_value, const double second_value) const;
};
}  // namespace localization
}  // namespace apollo
#endif  // MODULES_LOCALIZATION_LMD_LM_BAG_PROCESSOR_H_
