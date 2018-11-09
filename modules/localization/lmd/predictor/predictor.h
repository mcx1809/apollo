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
 * @file predictor.h
 * @brief The class of Predictor.
 */

#ifndef MODULES_LOCALIZATION_LMD_COMMON_PREDICTOR_PREDICTOR_H_
#define MODULES_LOCALIZATION_LMD_COMMON_PREDICTOR_PREDICTOR_H_

#include <map>
#include <set>
#include <string>

#include "modules/common/log.h"
#include "modules/common/status/status.h"
#include "modules/localization/lmd/common/pose_list.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

namespace {
constexpr char kPredictorOutputName[] = "output";
constexpr char kPredictorImu[] = "imu";
constexpr char kPredictorPerceptionName[] = "perception";
}  // namespace

/**
 * @class Predictor
 *
 * @brief  Interface for implementing predictor.
 */
class Predictor {
 public:
  explicit Predictor(double memory_cycle_sec_)
      : predicted_(memory_cycle_sec_) {}

  virtual ~Predictor() {}

  /**
   * @brief Get name of the predictor.
   * @return A name.
   */
  const std::string& Name() const { return name_; }

  /**
   * @brief Get names of the deps.
   * @return names of deps.
   */
  const std::set<std::string>& DepPredictors() const { return dep_names_; }

  /**
   * @brief Get predicted list.
   * @return Predicted list.
   */
  const PoseList& Predicted() const { return predicted_; }

  /**
   * @brief Update predicted list of deps.
   */
  void UpdateDepPredicted(const std::string& name, const PoseList& predicted) {
    UpdateDepPredicted(name, PoseList(predicted));
  }

  void UpdateDepPredicted(const std::string& name, PoseList&& predicted) {
    auto it = dep_names_.find(name);
    CHECK(it != dep_names_.end());

    dep_predicteds_.emplace(name, std::move(predicted));
  }

  /**
   * @brief Is predicted list updateable.
   * @return True if yes; no otherwise.
   */
  virtual bool Updateable() const;

  /**
   * @brief Predict a new pose and add it to predicted list.
   * @return Status::OK() if success; error otherwise.
   */
  virtual apollo::common::Status Update();

 protected:
  /**
   * @brief Initialization.
   * @param memory_cycle_sec The cycle of memory.
   */
  void Init(double memory_cycle_sec) {
    for (const auto& dep_name : dep_names_) {
      dep_predicteds_.emplace(dep_name, PoseList(memory_cycle_sec));
    }
  }

 protected:
  std::set<std::string> dep_names_;
  std::string name_;

  std::map<std::string, PoseList> dep_predicteds_;
  PoseList predicted_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_COMMON_PREDICTOR_PREDICTOR_H_
