/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file predictor_output.h
 * @brief The class of PredictorOutput.
 */

#ifndef MODULES_LOCALIZATION_LMD_PREDICTOR_OUTPUT_PREDICTOR_OUTPUT_H_
#define MODULES_LOCALIZATION_LMD_PREDICTOR_OUTPUT_PREDICTOR_OUTPUT_H_

#include <functional>
#include <vector>

#include "gtest/gtest.h"

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/localization/lmd/predictor/output/lmd_particle_filter.h"
#include "modules/localization/lmd/predictor/predictor.h"
/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class PredictorOutput
 *
 * @brief  Implementation of predictor.
 */
class PredictorOutput : public Predictor {
 public:
  explicit PredictorOutput(
      double memory_cycle_sec,
      const std::function<apollo::common::Status(double, const Pose &)>
          &publish_loc_func);
  virtual ~PredictorOutput();

  /**
   * @brief Overrided implementation of the virtual function "Updateable" in the
   * base class "Predictor".
   * @return True if yes; no otherwise.
   */
  bool Updateable() const override;

  /**
   * @brief Overrided implementation of the virtual function "Update" in the
   * base class "Predictor".
   * @return Status::OK() if success; error otherwise.
   */
  apollo::common::Status Update() override;

  /**
   * @brief Update lane markers from perception.
   * @param timestamp_sec The timestamp of lane markers.
   * @param lane_markers The lane markers.
   * @return True if success; false if not needed.
   */
  bool UpdateLaneMarkers(double timestamp_sec,
                         const apollo::perception::LaneMarkers &lane_markers);

 private:
  bool PredictByImu(double old_timestamp_sec, const Pose &old_pose,
                    double new_timestamp_sec, Pose *new_pose);
  bool PredictByParticleFiler(double old_timestamp_sec, const Pose &old_pose,
                              double new_timestamp_sec, Pose *new_pose);

 private:
  apollo::perception::LaneMarkers lane_markers_;
  double lane_markers_time_;
  std::function<apollo::common::Status(double, const Pose &)> publish_loc_func_;
  ParticleFilter pc_filter_;
  FRIEND_TEST(PredictorOutputTest, PredictByImu1);
  FRIEND_TEST(PredictorOutputTest, PredictByImu2);
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_PREDICTOR_OUTPUT_PREDICTOR_OUTPUT_H_
