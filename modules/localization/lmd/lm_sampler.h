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
 * @file lm_sampler.h
 * @brief The class of LMSampler.
 */

#ifndef MODULES_LOCALIZATION_LMD_LM_SAMPLER_H_
#define MODULES_LOCALIZATION_LMD_LM_SAMPLER_H_

#include <vector>

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/localization/lmd/pc_registrator.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class LMSampler
 *
 * @brief  Sampler for lane markers from percption.
 */
class LMSampler {
 public:
  /**
   * @brief  Update map for range.
   * @param lane_markers The lane marker from percption.
   * @return The sampling points.
   */
  std::vector<PCSourcePoint> Sampling(
      const apollo::perception::LaneMarkers& lane_markers);
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_LM_SAMPLER_H_
