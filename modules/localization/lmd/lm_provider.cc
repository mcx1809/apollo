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
  lane_index.first = std::numeric_limits<int>::max();
  lane_index.second = std::numeric_limits<int>::max();
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

void LMProvider::FindNearestLaneMarkerIndex(
    const apollo::common::PointENU& position) {
  CHECK(LaneMarkersPack_.lane_markers_size() != 0)
      << "Empty LaneMarkersPack from file " << FLAGS_lmd_rawinput_file;
  double distance = std::numeric_limits<double>::max();
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
        lane_index.first = lane_mark_pack_index;
        lane_index.second = lane_marker_index;
      }
    }
  }
}

void LMProvider::GetPrevLaneMarkerIndex() {
  CHECK(lane_index.first >= 0)
      << "index of current ContourOdometryLaneMarkers must not be less than 0 ";
  CHECK(lane_index.first < LaneMarkersPack_.lane_markers_size())
      << "index of current ContourOdometryLaneMarkers can not be greater than "
         "size of OdometryLaneMarkersPack";
  CHECK(lane_index.second <
        LaneMarkersPack_.lane_markers(lane_index.first).lane_marker_size())
      << "index of current OdometryLaneMarker can not be greater than size "
         "of ContourOdometryLaneMarkers";
  CHECK(lane_index.second > 0) << "To get prev lane marker, index of current "
                                  "OdometryLaneMarker must be greater than 0 ";
  lane_index.second -= 1;
}

void LMProvider::GetNextLaneMarkerIndex() {
  CHECK(lane_index.first >= 0)
      << "index of current ContourOdometryLaneMarkers must not be less than 0 ";
  CHECK(lane_index.first < LaneMarkersPack_.lane_markers_size())
      << "index of current ContourOdometryLaneMarkers can not be greater than "
         "size of OdometryLaneMarkersPack";
  CHECK(lane_index.second >= 0)
      << "index of current OdometryLaneMarker must not be less than 0 ";
  CHECK(lane_index.second <
        LaneMarkersPack_.lane_markers(lane_index.first).lane_marker_size() - 1)
      << "To get next lane marker,index of next OdometryLaneMarker must be "
         "less than size of ContourOdometryLaneMarkers";
  lane_index.second += 1;
}

void LMProvider::GetLeftLaneMarkerIndex() {
  CHECK(lane_index.first > 0)
      << "To get left lane marker, index of left ContourOdometryLaneMarkers "
         "must not be less than 0 ";
  CHECK(lane_index.first < LaneMarkersPack_.lane_markers_size())
      << "index of current ContourOdometryLaneMarkers can not be greater than "
         "size of OdometryLaneMarkersPack";
  CHECK(lane_index.second >= 0)
      << "index of current ContourOdometryLaneMarker must not be less than 0 ";
  CHECK(lane_index.second <
        LaneMarkersPack_.lane_markers(lane_index.first).lane_marker_size())
      << "index of current ContourOdometryLaneMarker can not be greater than "
         "size of ContourOdometryLaneMarkers";
  lane_index.first -= 1;
}

void LMProvider::GetRightLaneMarkerIndex() {
  CHECK(lane_index.first >= 0)
      << "index of current ContourOdometryLaneMarkers must not be less than 0 ";
  CHECK(lane_index.first < LaneMarkersPack_.lane_markers_size() - 1)
      << "to get right lane marker, index of right ContourOdometryLaneMarkers "
         "can not be greater than size of OdometryLaneMarkersPack";
  CHECK(lane_index.second >= 0)
      << "index of OdometryLaneMarker must not be less than 0 ";
  CHECK(lane_index.second <
        LaneMarkersPack_.lane_markers(lane_index.first).lane_marker_size() - 1)
      << "index of OdometryLaneMarker can not be greater than size "
         "of ContourOdometryLaneMarkers";
  lane_index.first -= 1;
}

const std::pair<int, int>& LMProvider::GetCurrentLaneMarkerIndex() const {
  return lane_index;
}

const apollo::localization::OdometryLaneMarker& LMProvider::GetLaneMarker()
    const {
  CHECK(lane_index.first >= 0)
      << "index of current ContourOdometryLaneMarkers must not be less than 0 ";
  CHECK(lane_index.first < LaneMarkersPack_.lane_markers_size())
      << "index of current ContourOdometryLaneMarkers can not be greater than "
         "size of OdometryLaneMarkersPack";
  CHECK(lane_index.second <
        LaneMarkersPack_.lane_markers(lane_index.first).lane_marker_size())
      << "index of current OdometryLaneMarker can not be greater than size "
         "of ContourOdometryLaneMarkers";
  CHECK(lane_index.second >= 0)
      << "index of current OdometryLaneMarker must not be less than 0 ";
  return LaneMarkersPack_.lane_markers(lane_index.first)
      .lane_marker(lane_index.second);
}

LMProvider::~LMProvider() {}

}  // namespace localization
}  // namespace apollo
