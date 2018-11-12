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

#include "modules/localization/lmd/predictor/raw/predictor_imu.h"

#include "modules/common/log.h"

namespace apollo {
namespace localization {

using apollo::common::Status;

PredictorImu::PredictorImu(double memory_cycle_sec)
    : Predictor(memory_cycle_sec) {
  name_ = kPredictorImu;
  on_adapter_thread = true;
}

PredictorImu::~PredictorImu() {}

bool PredictorImu::UpdateImu(const CorrectedImu& imu) {
  if (!imu.has_header() || !imu.header().has_timestamp_sec() ||
      !imu.has_imu()) {
    AERROR << "Message has not some feilds";
    return false;
  }

  auto timestamp_sec = imu.header().timestamp_sec();
  if (!predicted_.Push(timestamp_sec, imu.imu())) {
    AWARN << "Failed push pose to list, with timestamp[" << timestamp_sec
          << "]";
    return false;
  }

  return true;
}

bool PredictorImu::Updateable() const {
  return !predicted_.empty() && predicted_.Newer(latest_timestamp_sec_);
}

Status PredictorImu::Update() {
  if (!predicted_.empty()) {
    auto latest = predicted_.Latest();
    latest_timestamp_sec_ = latest->first;
  }
  return Status::OK();
}

}  // namespace localization
}  // namespace apollo
