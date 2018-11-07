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
 * @file tm_list.h
 * @brief The class of TimeMarkedList.
 */

#ifndef MODULES_LOCALIZATION_LMD_COMMON_TM_LIST_H_
#define MODULES_LOCALIZATION_LMD_COMMON_TM_LIST_H_

#include <algorithm>
#include <map>

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class TimeMarkedList
 *
 * @brief  A time marked list.
 */
template <class Data>
class TimeMarkedList : public std::map<double, Data> {
 public:
  typedef typename std::map<double, Data>::const_iterator Iterator;

  TimeMarkedList(double memory_cycle_sec = 0.0) {
    memory_cycle_sec_ = std::max(0.0, memory_cycle_sec);
  }

  bool Push(double timestamp_sec, const Data& data) {
    return Push(timestamp_sec, Data(data));
  }

  bool Push(double timestamp_sec, Data&& data) {
    auto latest = Latest();
    if (latest != this->end() && latest->first >= timestamp_sec) {
      return false;
    }

    this->emplace(timestamp_sec, std::move(data));

    auto oldest = Oldest();
    if (oldest != this->end()) {
      auto oldest1 = oldest;
      oldest1++;
      if (oldest1 != this->end() &&
          timestamp_sec - oldest->first > memory_cycle_sec_ &&
          timestamp_sec - oldest1->first >= memory_cycle_sec_) {
        this->erase(oldest);
      }
    }

    return true;
  }

  Iterator Latest() const {
    if (!this->empty()) {
      auto iter = this->end();
      iter--;
      return iter;
    } else {
      return this->end();
    }
  }

  Iterator Oldest() const { return this->begin(); }

 private:
  double memory_cycle_sec_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_COMMON_TM_LIST_H_
