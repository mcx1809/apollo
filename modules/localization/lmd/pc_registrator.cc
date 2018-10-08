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

#include "modules/localization/lmd/pc_registrator.h"

#include <limits>

#include "modules/common/log.h"

namespace apollo {
namespace localization {

using apollo::common::Point3D;
using apollo::common::PointENU;

PCRegistrator::PCRegistrator(PCMap* map) {
  CHECK_NOTNULL(map);
  map_ = map;
}

void PCRegistrator::Register(const std::vector<PCSourcePoint>& source_points,
                             const PointENU& position_estimated,
                             double heading_estimated, const PointENU* position,
                             double* heading) {}

double PCRegistrator::ComputeError(
    const std::vector<PCSourcePoint>& source_points,
    const apollo::common::PointENU& position_estimated,
    double heading_estimated) {
      
  return 0.0;
}

}  // namespace localization
}  // namespace apollo
