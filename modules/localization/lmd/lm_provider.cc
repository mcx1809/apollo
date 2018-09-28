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

#include "modules/localization/lmd/lm_provider.h"

#include <limits>

namespace apollo {
namespace localization {

LMProvider::LMProvider() {
  CHECK(apollo::common::util::GetProtoFromFile(FLAGS_lmd_rawinput_file,
                                               &LaneMarkersPack_))
      << "Unable to get raw lanemarkers from file " << FLAGS_lmd_rawinput_file;
}

double LMProvider::CalculateDistance(
    const apollo::common::PointENU& position,
    const apollo::common::PointENU& start_pos,
    const apollo::common::PointENU& end_pos) const {
  double ab = sqrt(pow((start_pos.x() - end_pos.x()), 2.0) +
                   pow((start_pos.y() - end_pos.y()), 2.0) +
                   pow((start_pos.z() - end_pos.z()), 2.0));
  double as = sqrt(pow((start_pos.x() - position.x()), 2.0) +
                   pow((start_pos.y() - position.y()), 2.0) +
                   pow((start_pos.z() - position.z()), 2.0));
  double bs = sqrt(pow((position.x() - end_pos.x()), 2.0) +
                   pow((position.y() - end_pos.y()), 2.0) +
                   pow((position.z() - end_pos.z()), 2.0));
  double cos_A = (pow(as, 2.0) + pow(ab, 2.0) - pow(bs, 2.0)) / (2 * ab * as);
  double sin_A = sqrt(1 - pow(cos_A, 2.0));
  return as * sin_A;
}

const std::pair<int, int> LMProvider::FindNearestLaneMarkerIndex(
    const apollo::common::PointENU& position) const {
  CHECK(LaneMarkersPack_.lane_markers_size() != 0)
      << "Empty LaneMarkersPack from file " << FLAGS_lmd_rawinput_file;
  double distance = std::numeric_limits<double>::max();
  std::pair<int, int> result(std::numeric_limits<int>::max(),
                             std::numeric_limits<int>::max());
  for (int lane_mark_pack_index = 0;
       lane_mark_pack_index < LaneMarkersPack_.lane_markers_size();
       lane_mark_pack_index++) {
    for (int lane_marker_index = 0;
         lane_marker_index <
         LaneMarkersPack_.lane_markers(lane_mark_pack_index).lane_marker_size();
         lane_marker_index++) {
      CHECK(LaneMarkersPack_.lane_markers(lane_mark_pack_index)
                .lane_marker(lane_marker_index)
                .has_start_position())
          << "No start position info in number " << lane_marker_index + 1
          << " lane marker in lane marker group " << (lane_mark_pack_index);
      CHECK(LaneMarkersPack_.lane_markers(lane_mark_pack_index)
                .lane_marker(lane_marker_index)
                .has_end_position())
          << "No end position info in number " << lane_marker_index + 1
          << " lane marker in lane marker group " << (lane_mark_pack_index);
      apollo::common::PointENU start_point =
          LaneMarkersPack_.lane_markers(lane_mark_pack_index)
              .lane_marker(lane_marker_index)
              .start_position();
      apollo::common::PointENU end_point =
          LaneMarkersPack_.lane_markers(lane_mark_pack_index)
              .lane_marker(lane_marker_index)
              .end_position();
      if (CalculateDistance(position, start_point, end_point) < distance) {
        distance = CalculateDistance(position, start_point, end_point);
        result.first = lane_mark_pack_index;
        result.second = lane_marker_index;
      }
    }
  }
  return result;
}

const std::pair<int, int> LMProvider::GetPrevLaneMarkerIndex(
    const std::pair<int, int>& current_index) const {
  CHECK(current_index.first >= 0)
      << "index of current ContourOdometryLaneMarkers must not be less than 0 ";
  CHECK(current_index.first < LaneMarkersPack_.lane_markers_size())
      << "index of current ContourOdometryLaneMarkers can not be greater than "
         "size of OdometryLaneMarkersPack";
  CHECK(current_index.second <
        LaneMarkersPack_.lane_markers(current_index.first).lane_marker_size())
      << "index of current OdometryLaneMarker can not be greater than size "
         "of ContourOdometryLaneMarkers";
  CHECK(current_index.second > 0)
      << "To get prev lane marker, index of current "
         "OdometryLaneMarker must be greater than 0 ";
  const std::pair<int, int> result(current_index.first,
                                   current_index.second - 1);
  return result;
}

const std::pair<int, int> LMProvider::GetNextLaneMarkerIndex(
    const std::pair<int, int>& current_index) const {
  CHECK(current_index.first >= 0)
      << "index of current ContourOdometryLaneMarkers must not be less than 0 ";
  CHECK(current_index.first < LaneMarkersPack_.lane_markers_size())
      << "index of current ContourOdometryLaneMarkers can not be greater than "
         "size of OdometryLaneMarkersPack";
  CHECK(current_index.second >= 0)
      << "index of current OdometryLaneMarker must not be less than 0 ";
  CHECK(current_index.second <
        LaneMarkersPack_.lane_markers(current_index.first).lane_marker_size() -
            1)
      << "To get next lane marker,index of next OdometryLaneMarker must be "
         "less than size of ContourOdometryLaneMarkers";
  const std::pair<int, int> result(current_index.first,
                                   current_index.second + 1);
  return result;
}

const std::pair<int, int> LMProvider::GetLeftLaneMarkerIndex(
    const std::pair<int, int>& current_index) const {
  CHECK(current_index.first > 0)
      << "to get left lane marker,index of current ContourOdometryLaneMarkers "
         "must not be less than 0 ";
  CHECK(current_index.first < LaneMarkersPack_.lane_markers_size())
      << "index of ContourOdometryLaneMarkers must be less than size of "
         "OdometryLaneMarkersPack";
  CHECK(current_index.second >= 0)
      << "index of OdometryLaneMarker must not be less than 0 ";
  CHECK(current_index.second <
        LaneMarkersPack_.lane_markers(current_index.first).lane_marker_size())
      << "index of OdometryLaneMarker must be less than size of "
         "ContourOdometryLaneMarkers";
  const std::pair<int, int> result(current_index.first - 1,
                                   current_index.second);
  return result;
}

const std::pair<int, int> LMProvider::GetRightLaneMarkerIndex(
    const std::pair<int, int>& current_index) const {
  CHECK(current_index.first >= 0)
      << "index of current ContourOdometryLaneMarkers must not be less than 0 ";
  CHECK(current_index.first < LaneMarkersPack_.lane_markers_size() - 1)
      << "to get right lane marker, index of right ContourOdometryLaneMarkers "
         "must be less than size of OdometryLaneMarkersPack";
  CHECK(current_index.second >= 0)
      << "index of OdometryLaneMarker must not be less than 0 ";
  CHECK(current_index.second <
        LaneMarkersPack_.lane_markers(current_index.first).lane_marker_size())
      << "index of OdometryLaneMarker can not be greater than size "
         "of ContourOdometryLaneMarkers";
  const std::pair<int, int> result(current_index.first + 1,
                                   current_index.second);
  return result;
}
const apollo::localization::OdometryLaneMarker& LMProvider::GetLaneMarker(
    const std::pair<int, int>& current_index) const {
  CHECK(current_index.first >= 0)
      << "index of current ContourOdometryLaneMarkers must not be less than 0";
  CHECK(current_index.first < LaneMarkersPack_.lane_markers_size())
      << "index of current ContourOdometryLaneMarkers can not be greater "
         "than size of OdometryLaneMarkersPack";
  CHECK(current_index.second <
        LaneMarkersPack_.lane_markers(current_index.first).lane_marker_size())
      << "index of current OdometryLaneMarker can not be greater than size "
         "of ContourOdometryLaneMarkers";
  CHECK(current_index.second >= 0)
      << "index of current OdometryLaneMarker must not be less than 0 ";
  return LaneMarkersPack_.lane_markers(current_index.first)
      .lane_marker(current_index.second);
}

LMProvider::~LMProvider() {}

}  // namespace localization
}  // namespace apollo
