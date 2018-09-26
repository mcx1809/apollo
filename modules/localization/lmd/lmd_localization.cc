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

#include "modules/localization/lmd/lmd_localization.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::ImuAdapter;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::time::Clock;
using ::Eigen::Vector3d;

LMDLocalization::LMDLocalization()
    : monitor_logger_(MonitorMessageItem::LOCALIZATION),
      map_offset_{FLAGS_map_offset_x, FLAGS_map_offset_y, FLAGS_map_offset_z} {}

LMDLocalization::~LMDLocalization() {}

Status LMDLocalization::Start() {
  AdapterManager::Init(FLAGS_lmd_adapter_config_file);

  // start ROS timer, one-shot = false, auto-start = true
  const double duration = 1.0 / FLAGS_localization_publish_freq;
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &LMDLocalization::OnTimer, this);
  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);

  // Add initialization of raw input

  tf2_broadcaster_.reset(new tf2_ros::TransformBroadcaster);

  return Status::OK();
}

Status LMDLocalization::Stop() {
  timer_.stop();
  return Status::OK();
}

void LMDLocalization::OnTimer(const ros::TimerEvent &event) {
  double time_delay =
      common::time::ToSecond(Clock::Now()) - last_received_timestamp_sec_;
  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  if (FLAGS_enable_gps_timestamp &&
      time_delay > FLAGS_gps_time_delay_tolerance) {
    buffer.ERROR() << "GPS message time delay: " << time_delay;
    buffer.PrintLog();
  }

  // Add msg_handler of raw input

  // publish localization messages
  PublishLocalization();
  service_started_ = true;

  // watch dog
  RunWatchDog();

  last_received_timestamp_sec_ = common::time::ToSecond(Clock::Now());
}

template <class T>
T LMDLocalization::InterpolateXYZ(const T &p1, const T &p2,
                                  const double frac1) {
  T p;
  double frac2 = 1.0 - frac1;
  if (p1.has_x() && !std::isnan(p1.x()) && p2.has_x() && !std::isnan(p2.x())) {
    p.set_x(p1.x() * frac2 + p2.x() * frac1);
  }
  if (p1.has_y() && !std::isnan(p1.y()) && p2.has_y() && !std::isnan(p2.y())) {
    p.set_y(p1.y() * frac2 + p2.y() * frac1);
  }
  if (p1.has_z() && !std::isnan(p1.z()) && p2.has_z() && !std::isnan(p2.z())) {
    p.set_z(p1.z() * frac2 + p2.z() * frac1);
  }
  return p;
}

void LMDLocalization::PrepareLocalizationMsg(
    LocalizationEstimate *localization) {
  // Add code to implement LocalizationEstimate msg generation
}

void LMDLocalization::PublishLocalization() {
  LocalizationEstimate localization;
  PrepareLocalizationMsg(&localization);

  // publish localization messages
  AdapterManager::PublishLocalization(localization);
  PublishPoseBroadcastTF(localization);
  ADEBUG << "[OnTimer]: Localization message publish success!";
}

void LMDLocalization::RunWatchDog() {
  if (!FLAGS_enable_watchdog) {
    return;
  }

  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);

  // Add code to implement watch dog
}

}  // namespace localization
}  // namespace apollo
