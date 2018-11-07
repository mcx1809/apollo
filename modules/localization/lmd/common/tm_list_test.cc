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

#include "modules/localization/lmd/common/tm_list.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {

TEST(TimeMarkedListTest, InsertAndFind) {
  TimeMarkedList<int> tm(100.0);

  for (auto i = 0; i < 200; ++i) {
    tm.Push((double)i, i);
  }

  EXPECT_EQ(101, tm.size());
  auto latest = tm.Latest();
  EXPECT_NE(tm.end(), latest);
  if (latest != tm.end()) {
    EXPECT_EQ(199, latest->second);
  }
  auto oldest = tm.Oldest();
  EXPECT_NE(tm.end(), oldest);
  if (oldest != tm.end()) {
    EXPECT_EQ(99, oldest->second);
  }
}

}  // namespace localization
}  // namespace apollo
