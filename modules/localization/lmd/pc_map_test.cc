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
namespace apollo {
namespace localization {
TEST(PCMapTest, UpdateRange) {
  LMProvider* lm_provider_ = new LMProvider();
  PCMap* map_ = new PCMap(lm_provider_);
  apollo::common::PointENU position;
  position.set_x(683092.86553);
  position.set_y(3110710.00623);
  position.set_z(66.24156);
  apollo::common::PointENU search_point;
  search_point.set_x(683091.66553);
  search_point.set_y(3110709.20623);
  search_point.set_z(66.24156);
  map_->UpdateRange(position, 16.0);
  auto nearest_point = map_->GetNearestPoint(search_point);
  EXPECT_NE(nullptr, nearest_point);
  EXPECT_NEAR(683091.07878110325, nearest_point->position.x(), 0.01);
  EXPECT_NEAR(3110709.6351002883, nearest_point->position.y(), 0.01);
  EXPECT_NEAR(57.115288617161426, nearest_point->position.z(), 0.01);
  delete map_;
}
}  // namespace localization
}  // namespace apollo
