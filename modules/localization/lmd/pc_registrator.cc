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
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace localization {

using apollo::common::Point3D;
using apollo::common::PointENU;
using apollo::common::math::RotateAxis;

namespace {
constexpr double kDistanceErrorRatio = 1.0;
constexpr double kDirectionErrorRatio = 1.0;
constexpr double kPositionXStep = 0.5;
constexpr double kPositionYStep = 0.5;
constexpr double kHeadingStep = 0.2;
constexpr int kPositionXStepHalfCount = 3;
constexpr int kPositionYStepHalfCount = 3;
constexpr int kHeadingStepHalfCount = 3;
constexpr double kPositionXOptStepMin = 0.01;
constexpr double kPositionYOptStepMin = 0.01;
constexpr double kHeadingOptStepMin = 0.01;
constexpr int kPositionXOptStepCount = 5;
constexpr int kPositionYOptStepCount = 5;
constexpr int kHeadingOptStepCount = 5;
}  // namespace

PCRegistrator::PCRegistrator(PCMap* map) {
  CHECK_NOTNULL(map);
  map_ = map;
}

void PCRegistrator::Register(const std::vector<PCSourcePoint>& source_points,
                             const PointENU& position_estimated,
                             double heading_estimated, PointENU* position,
                             double* heading) {
  CHECK(position_estimated.has_x());
  CHECK(position_estimated.has_y());
  CHECK_NOTNULL(position);
  CHECK_NOTNULL(heading);

  // find the near position and heading
  auto current_error = std::numeric_limits<double>::max();
  position->CopyFrom(position_estimated);
  *heading = heading_estimated;
  for (int i = -kHeadingStepHalfCount; i <= kHeadingStepHalfCount; ++i) {
    for (int j = -kPositionXStepHalfCount; j <= kPositionXStepHalfCount; ++j) {
      for (int k = -kPositionYStepHalfCount; k <= kPositionYStepHalfCount;
           ++k) {
        auto heading_testing = heading_estimated + i * kHeadingStep;
        PointENU position_testing;
        position_testing.set_x(position_estimated.x() + j * kPositionXStep);
        position_testing.set_y(position_estimated.y() + j * kPositionYStep);
        auto error =
            ComputeError(source_points, position_testing, heading_testing);
        if (error < current_error) {
          current_error = error;
          position->CopyFrom(position_testing);
          *heading = heading_testing;
        }
      }
    }
  }

  // optimize heading func
  auto optimize_heading_direct = [&](bool direct) {
    auto step = kHeadingOptStepMin;
    for (auto i = 0; i < kHeadingOptStepCount; ++i) {
      auto heading_testing = *heading + (direct ? 1.0 : -1.0) * step;
      auto error = ComputeError(source_points, *position, heading_testing);
      if (error < current_error) {
        current_error = error;
        *heading = heading_testing;
        step *= 2.0;
      } else {
        break;
      }
    }
  };

  // optimize position x func
  auto optimize_position_x_direct = [&](bool direct) {
    auto step = kPositionXOptStepMin;
    for (auto i = 0; i < kPositionXOptStepCount; ++i) {
      auto position_testing = *position;
      position_testing.set_x(position->x() + (direct ? 1.0 : -1.0) * step);
      auto error = ComputeError(source_points, position_testing, *heading);
      if (error < current_error) {
        current_error = error;
        position->set_x(position_testing.x());
        step *= 2.0;
      } else {
        break;
      }
    }
  };

  // optimize position y func
  auto optimize_position_y_direct = [&](bool direct) {
    auto step = kPositionYOptStepMin;
    for (auto i = 0; i < kPositionYOptStepCount; ++i) {
      auto position_testing = *position;
      position_testing.set_y(position->y() + (direct ? 1.0 : -1.0) * step);
      auto error = ComputeError(source_points, position_testing, *heading);
      if (error < current_error) {
        current_error = error;
        position->set_y(position_testing.y());
        step *= 2.0;
      } else {
        break;
      }
    }
  };

  // optimize position and heading
  optimize_heading_direct(true);
  optimize_heading_direct(false);
  optimize_position_x_direct(true);
  optimize_position_x_direct(false);
  optimize_position_y_direct(true);
  optimize_position_y_direct(false);
}

double PCRegistrator::ComputeError(
    const std::vector<PCSourcePoint>& source_points,
    const apollo::common::PointENU& position, double heading) {
  double error = 0.0;
  std::size_t not_found = 0;

  for (const auto& p : source_points) {
    // FLU to ENU
    double enu_x, enu_y;
    RotateAxis(-heading, p.position.x(), p.position.y(), &enu_x, &enu_y);
    enu_x += position.x();
    enu_y += position.y();

    // find a point from map
    PointENU enu_position;
    enu_position.set_x(enu_x);
    enu_position.set_y(enu_y);
    enu_position.set_z(0.0);
    auto nearest_p = map_->GetNearestPoint(enu_position);
    if (!nearest_p) {
      ++not_found;
      continue;
    }

    // get distance error
    auto x0 = nearest_p->position.x();
    auto y0 = nearest_p->position.y();
    auto xd = nearest_p->direction.x();
    auto yd = nearest_p->direction.y();
    auto xd2 = xd * xd;
    auto yd2 = yd * yd;
    auto r0 = yd * (enu_x - x0) - xd * (enu_y - y0);
    error += kDistanceErrorRatio * r0 * r0 / (xd2 + yd2);

    // FLU to ENU
    double enu_xd, enu_yd;
    RotateAxis(-heading, p.direction.x(), p.direction.y(), &enu_xd, &enu_yd);
    enu_xd += position.x();
    enu_yd += position.y();

    // get direction error
    auto r1 = xd * enu_xd + yd * enu_yd;
    error += (xd2 + yd2) * (enu_xd * enu_xd + enu_yd * enu_yd) - r1 * r1;
  }

  error *= static_cast<double>(source_points.size()) /
           (source_points.size() - not_found);

  return error;
}

}  // namespace localization
}  // namespace apollo
