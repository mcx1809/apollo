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

#include "modules/localization/lmd/lmd_localization.h"

#include <algorithm>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/util/thread_pool.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/localization/lmd/predictor/perception/predictor.h"

namespace apollo {
namespace localization {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::ChassisAdapter;
using apollo::common::adapter::GpsAdapter;
using apollo::common::adapter::ImuAdapter;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::util::ThreadPool;
using apollo::perception::PerceptionObstacles;

namespace {
constexpr double kDefaultMemoryCycle = 2.0;
constexpr int kDefaultThreadPoolSize = 2;
}  // namespace

LMDLocalization::LMDLocalization()
    : monitor_logger_(MonitorMessageItem::LOCALIZATION),
      map_offset_{FLAGS_map_offset_x, FLAGS_map_offset_y, FLAGS_map_offset_z} {}

LMDLocalization::~LMDLocalization() {}

Status LMDLocalization::Start() {
  // initialize predictors
  auto *predictor = new PredictorPerception(kDefaultMemoryCycle);
  predictors_.emplace(predictor->Name(), PredictorHandler(predictor));
  perception_ = &predictors_[predictor->Name()];

  // check predictors' dep
  for (const auto &p : predictors_) {
    auto &ph = p.second;
    for (const auto &dep_p : ph.predictor->DepPredicteds()) {
      if (predictors_.find(dep_p.first) == predictors_.end()) {
        AERROR << "Can not find predictor[" << dep_p.first << "]";
        predictors_.clear();
        return Status(
            ErrorCode::LOCALIZATION_ERROR,
            "Can not find predictor with name [\"" + dep_p.first + "\"]");
      }
    }
  }

  // initialize thread pool
  ThreadPool::Init(kDefaultThreadPoolSize);

  // initialize adapter manager
  AdapterManager::Init(FLAGS_lmd_adapter_config_file);
  AdapterManager::AddImuCallback(&LMDLocalization::OnImu, this);
  AdapterManager::AddGpsCallback(&LMDLocalization::OnGps, this);
  AdapterManager::AddChassisCallback(&LMDLocalization::OnChassis, this);
  AdapterManager::AddPerceptionObstaclesCallback(
      &LMDLocalization::OnPerceptionObstacles, this);

  // start ROS timer, one-shot = false, auto-start = true
  const double duration = 1.0 / FLAGS_localization_publish_freq;
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &LMDLocalization::OnTimer, this);

  tf2_broadcaster_.reset(new tf2_ros::TransformBroadcaster);

  return Status::OK();
}

Status LMDLocalization::Stop() {
  timer_.stop();
  ThreadPool::Stop();
  return Status::OK();
}

void LMDLocalization::OnImu(const CorrectedImu &imu) {
  // take a snapshot of the current received messages
  AdapterManager::Observe();
}

void LMDLocalization::OnGps(const Gps &gps) {
  // take a snapshot of the current received messages
  AdapterManager::Observe();
}

void LMDLocalization::OnChassis(const Chassis &chassis) {
  // take a snapshot of the current received messages
  AdapterManager::Observe();
}

void LMDLocalization::OnPerceptionObstacles(
    const PerceptionObstacles &obstacles) {
  if (!perception_->Busy()) {
    // take a snapshot of the current received messages
    AdapterManager::Observe();

    // update lane markers
    auto *adapter = AdapterManager::GetPerceptionObstacles();
    for (const auto &msg : *adapter) {
      if (!msg->has_header() || !msg->header().has_timestamp_sec() ||
          !msg->has_lane_marker()) {
        AERROR << "Message has not some feilds";
        continue;
      }

      auto *predictor =
          static_cast<PredictorPerception *>(perception_->predictor.get());
      if (!predictor->UpdateLaneMarkers(msg->header().timestamp_sec(),
                                        msg->lane_marker())) {
        break;
      }
    }

    // predicting
    Predicting();
  }
}

void LMDLocalization::OnTimer(const ros::TimerEvent &event) {
  // take a snapshot of the current received messages
  AdapterManager::Observe();

  // predicting
  Predicting();

  // watch dog
  RunWatchDog();
}

void LMDLocalization::Predicting() {
  bool finish = false;
  while (!finish) {
    finish = true;

    // update predicteds from deps
    for (auto &p : predictors_) {
      auto &ph = p.second;
      if (!ph.Busy()) {
        for (const auto &dep_p : ph.predictor->DepPredicteds()) {
          const auto &dep_name = dep_p.first;
          const auto &dep_ph = predictors_[dep_name];
          if (dep_ph.Busy()) {
            continue;
          }
          const auto &dep_predicted = dep_p.second;
          if (dep_predicted.Older(dep_ph.predictor->Predicted())) {
            ph.predictor->UpdateDepPredicted(dep_name,
                                             dep_ph.predictor->Predicted());
          }
        }
      }
    }

    // predict
    for (auto &p : predictors_) {
      auto &ph = p.second;
      if (!ph.Busy()) {
        if (ph.predictor->Updateable()) {
          finish = false;

          if (ph.predictor->UpdatingOnAdapterThread()) {
            ph.predictor->Update();
          } else {
            auto predictor = ph.predictor;
            ph.fut = ThreadPool::pool()->push(
                [=](int) mutable -> Status { return predictor->Update(); });
          }
        }
      }
    }
  }
}

void LMDLocalization::RunWatchDog() {
  if (!FLAGS_enable_watchdog) return;

  // Add code to implement watch dog
}

}  // namespace localization
}  // namespace apollo
