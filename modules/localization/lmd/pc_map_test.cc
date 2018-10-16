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

#include "gtest/gtest.h"

#include "modules/localization/lmd/lm_provider.h"

namespace apollo {
namespace localization {

using apollo::common::PointENU;

TEST(PCMapTest, GetNearestPoint) {
  LMProvider provider;
  PCMap map(&provider);
  PointENU position;
  position.set_x(683092.0);
  position.set_y(3110712.0);
  position.set_z(57.0);
  map.UpdateRange(position, 16.0);

  PointENU search_point;
  search_point.set_x(683092.0);
  search_point.set_y(3110712.0);
  search_point.set_z(57.0);
  auto nearest_point = map.GetNearestPoint(search_point);
  EXPECT_NE(nullptr, nearest_point);
  if (nearest_point != nullptr) {
    EXPECT_NEAR(683092.31, nearest_point->position.x(), 0.01);
    EXPECT_NEAR(3110712.43, nearest_point->position.y(), 0.01);
    EXPECT_NEAR(57.08, nearest_point->position.z(), 0.01);
  }
}

}  // namespace localization
}  // namespace apollo
