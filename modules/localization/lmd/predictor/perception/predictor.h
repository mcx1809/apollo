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
 * @file pose_predictor.h
 * @brief The class of PredictorPerception.
 */

#ifndef MODULES_LOCALIZATION_LMD_PREDICTOR_PERCEPTION_PREDICTOR_H_
#define MODULES_LOCALIZATION_LMD_PREDICTOR_PERCEPTION_PREDICTOR_H_

#include <mutex>

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/localization/lmd/common/tm_list.h"
#include "modules/localization/lmd/predictor/perception/lm_provider.h"
#include "modules/localization/lmd/predictor/perception/lm_sampler.h"
#include "modules/localization/lmd/predictor/perception/pc_map.h"
#include "modules/localization/lmd/predictor/perception/pc_registrator.h"
#include "modules/localization/lmd/predictor/predictor.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class PredictorPerception
 *
 * @brief  Implementation of predictor.
 */
class PredictorPerception : Predictor {
 public:
  PredictorPerception();
  virtual ~PredictorPerception();

  /**
   * @brief  Overrided implementation of the virtual function "Predict" in the
   * base class "Predictor".
   * @param old_pose The pose before prediction.
   * @param old_timestamp_sec The timestamp before prediction.
   * @param new_timestamp_sec The timestamp for prediction.
   * @param new_pose The predicted pose for output.
   * @return Status::OK() if success; error otherwise.
   */
  apollo::common::Status Predict(const Pose &old_pose, double old_timestamp_sec,
                                 double new_timestamp_sec,
                                 Pose *new_pose) override;

  /**
   * @brief Update lane markers from perception.
   * @param timestamp_sec The timestamp of lane markers.
   * @param lane_markers The lane markers.
   * @return Status::OK() if success; error otherwise.
   */
  apollo::common::Status UpdateLaneMarkers(
      double timestamp_sec,
      const apollo::perception::LaneMarkers &lane_markers);

 private:
  LMSampler lm_sampler_;
  LMProvider lm_provider_;
  PCMap pc_map_;
  PCRegistrator pc_registrator_;

  std::mutex mutex_;
  TimeMarkedList<std::vector<PCSourcePoint>> lane_markers_samples_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_PREDICTOR_PERCEPTION_PREDICTOR_H_
