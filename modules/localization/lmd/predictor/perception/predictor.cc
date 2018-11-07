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

#include "modules/localization/lmd/predictor/perception/predictor.h"

#include "modules/common/log.h"

#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using apollo::common::Status;
using apollo::perception::LaneMarkers;

namespace {
constexpr double kLaneMarkersSampleListDepth = 2.0;
}  // namespace

PredictorPerception::PredictorPerception()
    : pc_map_(FLAGS_enable_lmd_premapping ? &lm_provider_ : nullptr),
      pc_registrator_(&pc_map_),
      lane_markers_samples_(kLaneMarkersSampleListDepth) {}

PredictorPerception::~PredictorPerception() {}

Status PredictorPerception::Predict(const Pose& old_pose,
                                    double old_timestamp_sec,
                                    double new_timestamp_sec, Pose* new_pose) {
  CHECK_NOTNULL(new_pose);

  { auto lock = std::unique_lock<decltype(mutex_)>(mutex); }

  const auto& position_estimated = new_pose.position();
  auto heading_estimated = new_pose.heading();

  return Status::OK();
}

Status UpdateLaneMarkers(double timestamp_sec,
                         const LaneMarkers& lane_markers) {
  auto lane_markers_samples = lm_sampler_.Sampling(lane_markers);

  bool ret;
  {
    auto lock = std::unique_lock<decltype(mutex_)>(mutex);
    ret = lane_markers_samples_.Push(timestamp_sec, lane_markers_samples);
  }

  if (!ret) {
    AWARN << "Failed push lane_markers_samples to list, with timestamp["
          << timestamp_sec << "]";
  }

  return Status::OK();
}

}  // namespace localization
}  // namespace apollo
