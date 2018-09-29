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

#include "modules/localization/lmd/lm_matcher.h"
#include "modules/common/time/time.h"

namespace apollo {
namespace localization {

using apollo::common::time::Clock;

LMMatcher::LMMatcher(LMProvider* provider) {
  CHECK_NOTNULL(provider);

  provider_ = provider;
}

LMMatcher::~LMMatcher() {}

double LMMatcher::GetMold(const std::vector<double>& vec) {
  int n = vec.size();
  double sum = 0.0;
  for (int i = 0; i < n; ++i) sum += vec[i] * vec[i];
  return sqrt(sum);
}

double LMMatcher::GetSimilarity(const std::vector<double>& lhs,
                                const std::vector<double>& rhs) {
  int n = lhs.size();
  assert(n == (int)(rhs.size()));
  double tmp = 0.0;
  for (int i = 0; i < n; ++i) {
    tmp += lhs[i] * rhs[i];
  }
  return tmp / (GetMold(lhs) * GetMold(rhs));
}

double LMMatcher::Distance(double a[], double b[]) {
  double dis = 0.0;
  double x, y, x2, y2 = 0.0;
  x = a[0] - b[0];
  y = a[1] - b[1];
  x2 = x * x;
  y2 = y * y;
  dis = sqrt((x2 + y2));
  return dis;
}

int LMMatcher::Collinear(double a[], double b[], double c[]) {
  double k1, k2 = 0.0;
  double kx1, ky1, kx2, ky2 = 0.0;
  if (a[0] == b[0] && b[0] == c[0])
    return 1;
  else {
    kx1 = b[0] - a[0];
    kx2 = b[0] - c[0];
    ky1 = b[1] - a[1];
    ky2 = b[1] - a[1];
    k1 = ky1 / kx1;
    k2 = ky2 / kx2;
    if (k1 == k2)
      return 1;
    else
      return 0;
  }
}

double LMMatcher::Curvature(double a[], double b[], double c[]) {
  double cur = 0.0;
  if (1 == Collinear(a, b, c)) {
    cur = 0.0;
  } else {
    double radius = 0.0;
    double dis, dis1, dis2, dis3 = 0.0;
    double cosA = 0.0;
    dis1 = Distance(a, b);
    dis2 = Distance(a, c);
    dis3 = Distance(b, c);
    dis = dis2 * dis2 + dis3 * dis3 - dis1 * dis1;
    cosA = dis / (2 * dis2 * dis3);
    radius = 0.5 * dis1 / cosA;
    cur = 1 / radius;
  }
  return cur;
}

double LMMatcher::BestMatchScoreForTwoLane(
    const apollo::perception::LaneMarker& per_lane_marker,
    const apollo::localization::OdometryLaneMarker& map_lane_marker,
    double* bestmatch_yposition) {
  double c0_position_per = per_lane_marker.c0_position();
  double c1_heading_angle_per = per_lane_marker.c1_heading_angle();
  double c2_curvature_per = per_lane_marker.c2_curvature();
  double c3_curvature_derivative_per =
      per_lane_marker.c3_curvature_derivative();

  double c0_position_map = map_lane_marker.c0_position();
  double c1_heading_angle_map = map_lane_marker.c1_heading_angle();
  double c2_curvature_map = map_lane_marker.c2_curvature();
  double c3_curvature_derivative_map =
      map_lane_marker.c3_curvature_derivative();

  std::vector<double> curvature_per, curvature_map;

  curvature_per.push_back(0.0);
  curvature_map.push_back(0.0);
  double d_gap = 0.05;
  for (double y = d_gap; y < map_lane_marker.z_length(); y += d_gap) {
    double val_left[2] = {0.0, 0.0};
    val_left[1] = y - d_gap;
    val_left[0] = c3_curvature_derivative_map * pow(val_left[1], 3.0) +
                  c2_curvature_map * pow(val_left[1], 2.0) +
                  c1_heading_angle_map * val_left[1] + c0_position_map;

    double val_mid[2] = {0.0, 0.0};
    val_mid[1] = y;
    val_mid[0] = c3_curvature_derivative_map * pow(val_mid[1], 3.0) +
                 c2_curvature_map * pow(val_mid[1], 2.0) +
                 c1_heading_angle_map * val_mid[1] + c0_position_map;

    double val_right[2] = {0.0, 0.0};
    val_right[1] = y + d_gap;
    val_right[0] = c3_curvature_derivative_map * pow(val_right[1], 3.0) +
                   c2_curvature_map * pow(val_right[1], 2.0) +
                   c1_heading_angle_map * val_right[1] + c0_position_map;

    curvature_map.push_back(Curvature(val_left, val_mid, val_right));
  }
  curvature_map.push_back(0.0);

  for (double y = d_gap; y < 1; y += d_gap) {
    double val_left2[2] = {0.0, 0.0};
    val_left2[1] = y - d_gap;
    val_left2[0] = c3_curvature_derivative_per * pow(val_left2[1], 3.0) +
                   c2_curvature_per * pow(val_left2[1], 2.0) +
                   c1_heading_angle_per * val_left2[1] + c0_position_per;

    double val_mid2[2] = {0.0, 0.0};
    val_mid2[1] = y;
    val_mid2[0] = c3_curvature_derivative_per * pow(val_mid2[1], 3.0) +
                  c2_curvature_per * pow(val_mid2[1], 2.0) +
                  c1_heading_angle_per * val_mid2[1] + c0_position_per;

    double val_right2[2] = {0.0, 0.0};
    val_right2[1] = y + d_gap;
    val_right2[0] = c3_curvature_derivative_per * pow(val_right2[1], 3.0) +
                    c2_curvature_per * pow(val_right2[1], 2.0) +
                    c1_heading_angle_per * val_right2[1] + c0_position_per;

    curvature_per.push_back(Curvature(val_left2, val_mid2, val_right2));
  }
  curvature_per.push_back(0.0);

  int n_long = (int)(0.5 * (double)(curvature_per.size()));
  std::vector<double> curvature_per_part(n_long);
  std::copy(curvature_per.begin(), curvature_per.begin() + n_long,
            curvature_per_part.begin());

  double d_maxScore = 0.0;
  int n_pos = 0;
  std::vector<double> curvature_map_part(n_long);
  for (int i = 0; i < (int)(curvature_map.size()) - n_long; i++) {
    std::copy(curvature_map.begin() + i, curvature_per.begin() + i + n_long,
              curvature_map_part.begin());
    double d_Score = GetSimilarity(curvature_per_part, curvature_map_part);

    if (d_Score > d_maxScore) {
      d_maxScore = d_Score;
      n_pos = i;
    }
  }
  *bestmatch_yposition = n_pos * d_gap;
  return d_maxScore;
}

std::vector<OdometryLaneMarker> LMMatcher::MatchLaneMarkers(
    const apollo::common::PointENU& position_estimated,
    const apollo::perception::LaneMarkers& lane_markers, double timestamp) {
  std::vector<OdometryLaneMarker> match_lane_markers;
  match_lane_markers.clear();

  if (!(lane_markers.has_left_lane_marker() ||
        lane_markers.has_right_lane_marker() ||
        lane_markers.next_left_lane_marker_size() > 0 ||
        lane_markers.next_right_lane_marker_size() > 0)) {
    AERROR << "lane_markers is empty!";
    return match_lane_markers;
  }

  if (lane_markers.has_left_lane_marker()) {
    const auto& lane_maker = lane_markers.left_lane_marker();

    double d_maxScore = 0.0;
    int n_bestlane_flag = 1;
    double d_posy_cur, d_posy_prev, d_posy_next, d_bestPosy = 0.0;

    std::pair<int, int> index = provider_->FindNearestLaneMarkerIndex(position_estimated);
    const auto nearest_lane_marker = provider_->GetLaneMarker(index);
    double d_Score_cur =
        BestMatchScoreForTwoLane(lane_maker, *nearest_lane_marker, &d_posy_cur);
    if (d_Score_cur > d_maxScore) {
      n_bestlane_flag = 1;
      d_maxScore = d_Score_cur;
    }

    index = provider_->GetPrevLaneMarkerIndex(index);
    const auto prev_lane_marker = provider_->GetLaneMarker(index);
    double d_Score_prev =
        BestMatchScoreForTwoLane(lane_maker, *prev_lane_marker, &d_posy_prev);
    if (d_Score_prev > d_maxScore) {
      n_bestlane_flag = 2;
      d_maxScore = d_Score_prev;
    }

    index = provider_->GetNextLaneMarkerIndex(index);
    index = provider_->GetNextLaneMarkerIndex(index);
    const auto next_lane_marker = provider_->GetLaneMarker(index);
    double d_Score_next =
        BestMatchScoreForTwoLane(lane_maker, *next_lane_marker, &d_posy_next);
    if (d_Score_next > d_maxScore) {
      n_bestlane_flag = 3;
      d_maxScore = d_Score_next;
    }

    apollo::localization::OdometryLaneMarker bestodometrylanemarker;
    if (n_bestlane_flag == 1) {
      bestodometrylanemarker.CopyFrom(*nearest_lane_marker);
      d_bestPosy = d_posy_cur;

    } else if (n_bestlane_flag == 2) {
      bestodometrylanemarker.CopyFrom(*prev_lane_marker);
      d_bestPosy = d_posy_prev;

    } else {
      bestodometrylanemarker.CopyFrom(*next_lane_marker);
      d_bestPosy = d_posy_next;
    }

    match_lane_markers.push_back(bestodometrylanemarker);
  }

  return match_lane_markers;
}

}  // namespace localization
}  // namespace apollo