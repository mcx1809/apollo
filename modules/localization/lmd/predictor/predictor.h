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
 * @file predictor.h
 * @brief The class of Predictor.
 */

#ifndef MODULES_LOCALIZATION_LMD_COMMON_PREDICTOR_PREDICTOR_H_
#define MODULES_LOCALIZATION_LMD_COMMON_PREDICTOR_PREDICTOR_H_

#include "modules/localization/proto/localization.pb.h"

#include "modules/common/status/status.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class Predictor
 *
 * @brief  Interface for implementing predictor.
 */
class Predictor {
 public:
  virtual ~Predictor() {}

  /**
   * @brief Estimate pose with specific timestamp.
   * @param old_pose The pose before prediction.
   * @param old_timestamp_sec The timestamp before prediction.
   * @param new_timestamp_sec The timestamp for prediction.
   * @param new_pose The pose to prediction.
   * @return Status::OK() if success; error otherwise.
   */
  virtual apollo::common::Status Predict(const Pose &old_pose,
                                         double old_timestamp_sec,
                                         double new_timestamp_sec,
                                         Pose *new_pose, double *new_timestamp);
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_COMMON_PREDICTOR_PREDICTOR_H_
