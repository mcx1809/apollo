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

#include "modules/localization/lmd/pc_map.h"

#include "modules/common/log.h"

namespace apollo {
namespace localization {

using apollo::common::Point3D;
using apollo::common::PointENU;
using apollo::common::Status;

PCMap::PCMap(LMProvider* provider) {
  CHECK_NOTNULL(provider);
  provider_ = provider;
}

Status PCMap::UpdateRange(const PointENU& position, double radius) {
  return Status::OK();
}

PCMapPoint* PCMap::GetMatchedPoint(const PointENU& position) { return nullptr; }

}  // namespace localization
}  // namespace apollo
