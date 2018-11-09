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

#include "modules/localization/lmd/lm_bag_processor.h"

namespace apollo {
namespace localization {

using apollo::common::Point3D;
using apollo::common::PointENU;
using apollo::common::math::RotateAxis;
using apollo::localization::LocalizationEstimate;
using apollo::localization::OdometryLaneMarkersPack;
using apollo::perception::PerceptionObstacles;

namespace {
constexpr float kEpsilon = 0.00001;
}  // namespace

LMProcessor::LMProcessor(const std::string filename) {
  const std::vector<std::string> topics_{FLAGS_perception_obstacle_topic,
                                         FLAGS_localization_topic};
  rosbag::Bag bag;
  try {
    bag.open(filename, rosbag::bagmode::Read);
  } catch (const rosbag::BagIOException& e) {
    AERROR << "BagIOException when open bag: " << filename
           << " Exception: " << e.what();
    bag.close();
    return;
  } catch (...) {
    AERROR << "Failed to open bag: " << filename;
    bag.close();
    return;
  }
  rosbag::View view(bag, rosbag::TopicQuery(topics_));
  for (auto it = view.begin(); it != view.end(); ++it) {
    if (it->getTopic() == FLAGS_localization_topic) {
      localization_msgs_.push_back(*it->instantiate<LocalizationEstimate>());
    } else if (it->getTopic() == FLAGS_perception_obstacle_topic) {
      obstacle_msgs_.push_back(*it->instantiate<PerceptionObstacles>());
    }
  }
  bag.close();
}

bool LMProcessor::IsDoubleEqual(const double first_value,
                                const double second_value) const {
  if (std::abs(first_value - second_value) <= kEpsilon) return true;
  return false;
}

bool LMProcessor::InterpolatePose(const double timestamp_sec,
                                  PointENU* location_ptr,
                                  double* heading) const {
  for (auto it = localization_msgs_.begin(); it != localization_msgs_.end();
       ++it) {
    if (!(it->has_header() && it->header().has_timestamp_sec() &&
          it->has_pose() && it->pose().has_position())) {
      AERROR << "localization_msgs_ has no header or no some fields";
      return false;
    }
    auto before_iter = it;
    if (IsDoubleEqual(before_iter->header().timestamp_sec(), timestamp_sec)) {
      *location_ptr = before_iter->pose().position();
      *heading = before_iter->pose().heading();
      return true;
    }
    if (++it != localization_msgs_.end()) {
      if (IsDoubleEqual(it->header().timestamp_sec(), timestamp_sec)) {
        *location_ptr = it->pose().position();
        *heading = it->pose().heading();
        return true;
      }
      if (before_iter->header().timestamp_sec() < timestamp_sec &&
          timestamp_sec < it->header().timestamp_sec()) {
        double proportion =
            (timestamp_sec - before_iter->header().timestamp_sec()) /
            (it->header().timestamp_sec() - timestamp_sec);
        location_ptr->set_x(before_iter->pose().position().x() * proportion +
                            it->pose().position().x() * (1 - proportion));
        location_ptr->set_y(before_iter->pose().position().y() * proportion +
                            it->pose().position().y() * (1 - proportion));
        location_ptr->set_z(before_iter->pose().position().z() * proportion +
                            it->pose().position().z() * (1 - proportion));
        *heading = before_iter->pose().heading() * proportion +
                   it->pose().heading() * (1 - proportion);
        return true;
      }
    }
  }
  return false;
}

void LMProcessor::SerializeToFile(
    const OdometryLaneMarkersPack* lane_markers_pack_ptr) const {
  apollo::common::util::SetProtoToBinaryFile(*lane_markers_pack_ptr,
                                             FLAGS_lmd_rawinput_bin_file);
}

double LMProcessor::GetCurveVal(const double x_value, const double c0,
                                const double c1, const double c2,
                                const double c3) const {
  return c3 * pow(x_value, 3.0) + c2 * pow(x_value, 2.0) + c1 * x_value + c0;
}

double LMProcessor::GetDerivative(const double x_value, const double c0,
                                  const double c1, const double c2,
                                  const double c3) const {
  return 3 * c3 * pow(x_value, 2.0) + 2 * c2 + c1;
}

double LMProcessor::GetCurvity(const double x_value, const double c0,
                               const double c1, const double c2,
                               const double c3) const {
  double derivative = GetDerivative(x_value, c0, c1, c2, c3);
  return abs(6 * c3 * x_value + 2 * c2) /
         pow(1 + pow(derivative, 2.0), (3.0 / 2));
}

void LMProcessor::PrepareMarkersPack(
    OdometryLaneMarkersPack* lane_markers_pack) const {
  auto left_lane_marker_group = lane_markers_pack->add_lane_markers();
  auto right_lane_marker_group = lane_markers_pack->add_lane_markers();
  for (auto it = obstacle_msgs_.begin(); it != obstacle_msgs_.end(); ++it) {
    if (!it->has_lane_marker()) {
      AERROR << "No lane markers info in this perception obstacle";
      continue;
    }
    PointENU location, result_location;
    Point3D direct;
    double heading;
    if (!InterpolatePose(it->header().timestamp_sec(), &location, &heading)) {
      AERROR << "given timestamp_sec has no suitable location";
      continue;
    }
    if (it->lane_marker().has_left_lane_marker()) {
      auto left_lm_ = left_lane_marker_group->add_lane_marker();
      auto c0_l_ = it->lane_marker().left_lane_marker().c0_position();
      auto c1_l_ = it->lane_marker().left_lane_marker().c1_heading_angle();
      auto c2_l_ = it->lane_marker().left_lane_marker().c2_curvature();
      auto c3_l_ =
          it->lane_marker().left_lane_marker().c3_curvature_derivative();
      for (int point_size = 0; point_size < 10; point_size++) {
        auto point = left_lm_->add_points();
        double enu_x, enu_y;
        RotateAxis(-heading, point_size * 0.1,
                   GetCurveVal(point_size * 0.1, c0_l_, c1_l_, c2_l_, c3_l_),
                   &enu_x, &enu_y);
        enu_x += location.x();
        enu_y += location.y();
        result_location.set_x(enu_x);
        result_location.set_y(enu_y);
        result_location.set_z(location.z());
        point->mutable_position()->CopyFrom(result_location);
        RotateAxis(-heading, 1.0, 0.0, &enu_x, &enu_y);
        direct.set_x(enu_x);
        direct.set_y(enu_y);
        direct.set_z(0.0);
        point->mutable_direct()->CopyFrom(direct);
        point->set_curvature(
            GetCurvity(point_size * 0.1, c0_l_, c1_l_, c2_l_, c3_l_));
      }
    }
    if (it->lane_marker().has_right_lane_marker()) {
      auto right_lm_ = right_lane_marker_group->add_lane_marker();
      auto c0_r_ = it->lane_marker().right_lane_marker().c0_position();
      auto c1_r_ = it->lane_marker().right_lane_marker().c1_heading_angle();
      auto c2_r_ = it->lane_marker().right_lane_marker().c2_curvature();
      auto c3_r_ =
          it->lane_marker().right_lane_marker().c3_curvature_derivative();
      for (int point_size = 0; point_size < 10; point_size++) {
        auto point = right_lm_->add_points();
        double enu_x, enu_y;
        RotateAxis(-heading, point_size * 0.1,
                   GetCurveVal(point_size * 0.1, c0_r_, c1_r_, c2_r_, c3_r_),
                   &enu_x, &enu_y);
        enu_x += location.x();
        enu_y += location.y();
        result_location.set_x(enu_x);
        result_location.set_y(enu_y);
        result_location.set_z(location.z());
        point->mutable_position()->CopyFrom(result_location);
        RotateAxis(-heading, 1.0, 0.0, &enu_x, &enu_y);
        direct.set_x(enu_x);
        direct.set_y(enu_y);
        direct.set_z(0.0);
        point->mutable_direct()->CopyFrom(direct);
        point->set_curvature(
            GetCurvity(point_size * 0.1, c0_r_, c1_r_, c2_r_, c3_r_));
      }
    }
  }
}

}  // namespace localization
}  // namespace apollo
