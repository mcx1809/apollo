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

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/util/thread_pool.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/localization/lmd/predictor/output/predictor.h"
#include "modules/localization/lmd/predictor/perception/predictor.h"
#include "modules/localization/lmd/predictor/raw/predictor_gps.h"
#include "modules/localization/lmd/predictor/raw/predictor_imu.h"

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
    : monitor_logger_(MonitorMessageItem::LOCALIZATION) {}

LMDLocalization::~LMDLocalization() {}

Status LMDLocalization::Start() {
  // initialize predictors
  Predictor *predictor = new PredictorGps(kDefaultMemoryCycle);
  predictors_.emplace(predictor->Name(), PredictorHandler(predictor));
  gps_ = &predictors_[predictor->Name()];
  predictor = new PredictorImu(kDefaultMemoryCycle);
  predictors_.emplace(predictor->Name(), PredictorHandler(predictor));
  imu_ = &predictors_[predictor->Name()];
  predictor = new PredictorPerception(kDefaultMemoryCycle);
  predictors_.emplace(predictor->Name(), PredictorHandler(predictor));
  perception_ = &predictors_[predictor->Name()];
  predictor = new PredictorOutput(
      kDefaultMemoryCycle, [&](const LocalizationEstimate &localization) {
        AdapterManager::PublishLocalization(localization);
        PublishPoseBroadcastTF(localization);
        return Status::OK();
      });
  predictors_.emplace(predictor->Name(), PredictorHandler(predictor));
  output_ = &predictors_[predictor->Name()];

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
  if (!imu_->Busy()) {
    // take a snapshot of the current received messages
    AdapterManager::Observe();

    // update messages
    auto *adapter = AdapterManager::GetImu();
    for (const auto &msg : *adapter) {
      auto *predictor = static_cast<PredictorImu *>(imu_->predictor.get());
      if (!predictor->UpdateImu(*msg)) {
        break;
      }
    }

    // predicting
    Predicting();
  }
}

void LMDLocalization::OnGps(const Gps &gps) {
  if (!gps_->Busy()) {
    // take a snapshot of the current received messages
    AdapterManager::Observe();

    // update messages
    auto *adapter = AdapterManager::GetGps();
    for (const auto &msg : *adapter) {
      auto *predictor = static_cast<PredictorGps *>(gps_->predictor.get());
      if (!predictor->UpdateGps(*msg)) {
        break;
      }
    }

    // predicting
    Predicting();
  }
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
            ph.fut =
                ThreadPool::pool()->push([=](int thread_id) mutable -> Status {
                  return predictor->Update();
                });
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
