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
 * @file pc_register.h
 * @brief The class of PCRegister.
 */

#ifndef MODULES_LOCALIZATION_LMD_PC_REGISTRATOR_H_
#define MODULES_LOCALIZATION_LMD_PC_REGISTRATOR_H_

#include <vector>

#include "modules/common/proto/geometry.pb.h"

#include "modules/localization/lmd/pc_map.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @struct PCSourcePoint
 *
 * @brief  Point used to match for points of map.
 */
struct PCSourcePoint {
  apollo::common::PointENU position;
  apollo::common::Point3D direction;
  double curvature;
};

/**
 * @class PCRegistrator
 *
 * @brief  Register the position and heading.
 */
class PCRegistrator {
 public:
  explicit PCRegistrator(PCMap* map);

  /**
   * @brief  Register position and heading.
   * @param source_points Points .
   * @param position_estimated The estimated position.
   * @param heading_estimated The estimated heading.
   * @param position The accurate position.
   * @param heading The accurate heading.
   */
  void Register(const std::vector<PCSourcePoint>& source_points,
                const apollo::common::PointENU& position_estimated,
                double heading_estimated,
                const apollo::common::PointENU* position, double* heading);

 private:
  PCMap* map_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_PC_REGISTRATOR_H_
