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
 * @file lmd_localization.h
 * @brief The class of LMDLocalization.
 */

#ifndef MODULES_LOCALIZATION_LMD_LMD_LOCALIZATION_H_
#define MODULES_LOCALIZATION_LMD_LMD_LOCALIZATION_H_

#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ros/include/ros/ros.h"

#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/localization/lmd/lm_provider.h"
#include "modules/localization/lmd/lm_sampler.h"
#include "modules/localization/lmd/pc_map.h"
#include "modules/localization/lmd/pc_registrator.h"
#include "modules/localization/localization_base.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class LMDLocalization
 *
 * @brief Generate localization info based on LMD.
 */
class LMDLocalization : public LocalizationBase {
 public:
  LMDLocalization();
  virtual ~LMDLocalization();

  /**
   * @brief Module start function.
   * @return Start status.
   */
  apollo::common::Status Start() override;

  /**
   * @brief Module stop function.
   * @return Stop status.
   */
  apollo::common::Status Stop() override;

 private:
  void OnGps(const localization::Gps &gps);
  void OnPerceptionObstacles(
      const apollo::perception::PerceptionObstacles &obstacles);
  void OnTimer(const ros::TimerEvent &event);
  void PrepareLocalizationMsg(LocalizationEstimate *localization);
  bool FindMatchingIMU(const double timestamp_sec, CorrectedImu *imu_msg);
  void RunWatchDog();

 private:
  ros::Timer timer_;
  apollo::common::monitor::MonitorLogger monitor_logger_;
  const std::vector<double> map_offset_;
  LMProvider lm_provider_;
  LMSampler lm_sampler_;
  PCMap pc_map_;
  PCRegistrator pc_registrator_;
  bool has_last_pose_ = false;
  apollo::localization::Pose last_pose_;
  double last_pose_timestamp_sec_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_LMD_LOCALIZATION_H_
