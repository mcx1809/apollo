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

#include "modules/localization/lmd/lmd_localization.h"

#include "glog/logging.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using apollo::common::Point3D;
using apollo::common::PointENU;
using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::ImuAdapter;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::time::Clock;
using apollo::perception::PerceptionObstacles;

namespace {
constexpr double kPCMapSearchRadius = 10.0;
}  // namespace

template <class T>
static T InterpolateXYZ(const T &p1, const T &p2, const double frac1) {
  T p;
  double frac2 = 1.0 - frac1;
  if (p1.has_x() && !std::isnan(p1.x()) && p2.has_x() && !std::isnan(p2.x())) {
    p.set_x(p1.x() * frac2 + p2.x() * frac1);
  }
  if (p1.has_y() && !std::isnan(p1.y()) && p2.has_y() && !std::isnan(p2.y())) {
    p.set_y(p1.y() * frac2 + p2.y() * frac1);
  }
  if (p1.has_z() && !std::isnan(p1.z()) && p2.has_z() && !std::isnan(p2.z())) {
    p.set_z(p1.z() * frac2 + p2.z() * frac1);
  }
  return p;
}

static bool InterpolateIMU(const CorrectedImu &imu1, const CorrectedImu &imu2,
                           const double timestamp_sec, CorrectedImu *imu_msg) {
  if (!(imu1.has_header() && imu1.header().has_timestamp_sec() &&
        imu2.has_header() && imu2.header().has_timestamp_sec())) {
    AERROR << "imu1 and imu2 has no header or no timestamp_sec in header";
    return false;
  }
  if (timestamp_sec - imu1.header().timestamp_sec() <
      FLAGS_timestamp_sec_tolerance) {
    AERROR << "[InterpolateIMU]: the given time stamp[" << timestamp_sec
           << "] is older than the 1st message["
           << imu1.header().timestamp_sec() << "]";
    *imu_msg = imu1;
  } else if (timestamp_sec - imu2.header().timestamp_sec() >
             FLAGS_timestamp_sec_tolerance) {
    AERROR << "[InterpolateIMU]: the given time stamp[" << timestamp_sec
           << "] is newer than the 2nd message["
           << imu2.header().timestamp_sec() << "]";
    *imu_msg = imu1;
  } else {
    *imu_msg = imu1;
    imu_msg->mutable_header()->set_timestamp_sec(timestamp_sec);

    double time_diff =
        imu2.header().timestamp_sec() - imu1.header().timestamp_sec();
    if (fabs(time_diff) >= 0.001) {
      double frac1 =
          (timestamp_sec - imu1.header().timestamp_sec()) / time_diff;

      if (imu1.has_imu() && imu1.imu().has_angular_velocity() &&
          imu2.has_imu() && imu2.imu().has_angular_velocity()) {
        auto val = InterpolateXYZ(imu1.imu().angular_velocity(),
                                  imu2.imu().angular_velocity(), frac1);
        imu_msg->mutable_imu()->mutable_angular_velocity()->CopyFrom(val);
      }

      if (imu1.has_imu() && imu1.imu().has_linear_acceleration() &&
          imu2.has_imu() && imu2.imu().has_linear_acceleration()) {
        auto val = InterpolateXYZ(imu1.imu().linear_acceleration(),
                                  imu2.imu().linear_acceleration(), frac1);
        imu_msg->mutable_imu()->mutable_linear_acceleration()->CopyFrom(val);
      }

      if (imu1.has_imu() && imu1.imu().has_euler_angles() && imu2.has_imu() &&
          imu2.imu().has_euler_angles()) {
        auto val = InterpolateXYZ(imu1.imu().euler_angles(),
                                  imu2.imu().euler_angles(), frac1);
        imu_msg->mutable_imu()->mutable_euler_angles()->CopyFrom(val);
      }
    }
  }
  return true;
}

static void FillPoseFromImu(const Pose &imu, Pose *pose) {
  // linear acceleration
  if (imu.has_linear_acceleration()) {
    if (FLAGS_enable_map_reference_unify) {
      if (pose->has_orientation()) {
        // linear_acceleration:
        // convert from vehicle reference to map reference
        Eigen::Vector3d orig(imu.linear_acceleration().x(),
                             imu.linear_acceleration().y(),
                             imu.linear_acceleration().z());
        auto vec = common::math::QuaternionRotate(pose->orientation(), orig);
        pose->mutable_linear_acceleration()->set_x(vec[0]);
        pose->mutable_linear_acceleration()->set_y(vec[1]);
        pose->mutable_linear_acceleration()->set_z(vec[2]);

        // linear_acceleration_vfr
        pose->mutable_linear_acceleration_vrf()->CopyFrom(
            imu.linear_acceleration());
      } else {
        AERROR << "[PrepareLocalizationMsg]: "
               << "fail to convert linear_acceleration";
      }
    } else {
      pose->mutable_linear_acceleration()->CopyFrom(imu.linear_acceleration());
    }
  }

  // angular velocity
  if (imu.has_angular_velocity()) {
    if (FLAGS_enable_map_reference_unify) {
      if (pose->has_orientation()) {
        // angular_velocity:
        // convert from vehicle reference to map reference
        Eigen::Vector3d orig(imu.angular_velocity().x(),
                             imu.angular_velocity().y(),
                             imu.angular_velocity().z());
        auto vec = common::math::QuaternionRotate(pose->orientation(), orig);
        pose->mutable_angular_velocity()->set_x(vec[0]);
        pose->mutable_angular_velocity()->set_y(vec[1]);
        pose->mutable_angular_velocity()->set_z(vec[2]);

        // angular_velocity_vf
        pose->mutable_angular_velocity_vrf()->CopyFrom(imu.angular_velocity());
      } else {
        AERROR << "[PrepareLocalizationMsg]: "
               << "fail to convert angular_velocity";
      }
    } else {
      pose->mutable_angular_velocity()->CopyFrom(imu.angular_velocity());
    }
  }

  // euler angle
  if (imu.has_euler_angles())
    pose->mutable_euler_angles()->CopyFrom(imu.euler_angles());
}

LMDLocalization::LMDLocalization()
    : monitor_logger_(MonitorMessageItem::LOCALIZATION),
      map_offset_{FLAGS_map_offset_x, FLAGS_map_offset_y, FLAGS_map_offset_z},
      pc_map_(&lm_provider_),
      pc_registrator_(&pc_map_) {}

LMDLocalization::~LMDLocalization() {}

Status LMDLocalization::Start() {
  AdapterManager::Init(FLAGS_lmd_adapter_config_file);

  // Perception Obstacles
  AdapterManager::AddPerceptionObstaclesCallback(
      &LMDLocalization::OnPerceptionObstacles, this);

  // start ROS timer, one-shot = false, auto-start = true
  const double duration = 1.0 / FLAGS_localization_publish_freq;
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &LMDLocalization::OnTimer, this);

  tf2_broadcaster_.reset(new tf2_ros::TransformBroadcaster);

  return Status::OK();
}

Status LMDLocalization::Stop() {
  timer_.stop();
  return Status::OK();
}

void LMDLocalization::OnGps(const localization::Gps &gps) {
  if (!gps.has_header() || !gps.header().has_timestamp_sec() ||
      !gps.has_localization() || !gps.localization().has_position()) {
    AERROR << "gps has no header or no some fields";
    return;
  }

  auto timestamp = gps.header().timestamp_sec();
  const auto &pose = gps.localization();
  if (!has_last_pose_ || last_pose_timestamp_sec_ < timestamp) {
    if (!has_last_pose_) AINFO << "initialize pose";

    Pose new_pose;

    // position
    // world frame -> map frame
    new_pose.mutable_position()->set_x(pose.position().x() - map_offset_[0]);
    new_pose.mutable_position()->set_y(pose.position().y() - map_offset_[1]);
    new_pose.mutable_position()->set_z(pose.position().z() - map_offset_[2]);

    // orientation
    if (pose.has_orientation()) {
      new_pose.mutable_orientation()->CopyFrom(pose.orientation());
      auto heading = common::math::QuaternionToHeading(
          pose.orientation().qw(), pose.orientation().qx(),
          pose.orientation().qy(), pose.orientation().qz());
      new_pose.set_heading(heading);
    }

    // linear velocity
    if (pose.has_linear_velocity())
      new_pose.mutable_linear_velocity()->CopyFrom(pose.linear_velocity());

    // IMU
    CorrectedImu imu_msg;
    if (!FindMatchingIMU(timestamp, &imu_msg)) return;
    CHECK(imu_msg.has_imu());
    const auto &imu = imu_msg.imu();
    FillPoseFromImu(imu, &new_pose);

    has_last_pose_ = true;
    last_pose_.CopyFrom(new_pose);
    last_pose_timestamp_sec_ = timestamp;
  }
}

void LMDLocalization::OnPerceptionObstacles(
    const PerceptionObstacles &obstacles) {
  if (has_last_pose_ && obstacles.has_lane_marker()) {
    if (!obstacles.has_header() || !obstacles.header().has_timestamp_sec()) {
      AERROR << "obstacles has no header or no some fields";
      return;
    }

    auto timestamp = obstacles.header().timestamp_sec();
    if (last_pose_timestamp_sec_ > timestamp) return;

    // TODO(all):
    const auto &position_estimated = last_pose_.position();
    auto heading_estimated = last_pose_.heading();

    // update pc_map
    if (pc_map_.UpdateRange(position_estimated, kPCMapSearchRadius) !=
        Status::OK()) {
      AERROR << "update pc_map failed";
      return;
    }

    // sampling lane markers
    const auto &lane_markers = obstacles.lane_marker();
    auto source_points = lm_sampler_.Sampling(lane_markers);

    // point cloud registration
    PointENU position;
    double heading;
    pc_registrator_.Register(source_points, position_estimated,
                             heading_estimated, &position, &heading);

    Pose new_pose;

    // position
    // world frame -> map frame
    new_pose.mutable_position()->set_x(position.x() - map_offset_[0]);
    new_pose.mutable_position()->set_y(position.y() - map_offset_[1]);
    new_pose.mutable_position()->set_z(position.z() - map_offset_[2]);

    // heading
    new_pose.set_heading(heading);

    // linear velocity
    // TODO(all):

    // IMU
    CorrectedImu imu_msg;
    if (!FindMatchingIMU(timestamp, &imu_msg)) return;
    CHECK(imu_msg.has_imu());
    const auto &imu = imu_msg.imu();
    FillPoseFromImu(imu, &new_pose);

    has_last_pose_ = true;
    last_pose_.CopyFrom(new_pose);
    last_pose_timestamp_sec_ = timestamp;
  }
}

void LMDLocalization::OnTimer(const ros::TimerEvent &event) {
  if (!has_last_pose_) return;

  // take a snapshot of the current received messages
  AdapterManager::Observe();

  //  Prepare localization message.
  LocalizationEstimate localization;
  PrepareLocalizationMsg(&localization);

  // publish localization messages
  AdapterManager::PublishLocalization(localization);
  PublishPoseBroadcastTF(localization);
  ADEBUG << "[OnTimer]: Localization message publish success!";

  // watch dog
  RunWatchDog();
}

void LMDLocalization::PrepareLocalizationMsg(
    LocalizationEstimate *localization) {
  localization->Clear();

  // header
  AdapterManager::FillLocalizationHeader(FLAGS_localization_module_name,
                                         localization);

  // TODO(all):

  // measurement_time
  localization->set_measurement_time(last_pose_timestamp_sec_);

  // pose
  localization->mutable_pose()->CopyFrom(last_pose_);
}

bool LMDLocalization::FindMatchingIMU(const double timestamp_sec,
                                      CorrectedImu *imu_msg) {
  auto *imu_adapter = AdapterManager::GetImu();
  if (imu_adapter->Empty()) {
    AERROR << "Cannot find Matching IMU. "
           << "IMU message Queue is empty! timestamp[" << timestamp_sec << "]";
    return false;
  }

  // scan imu buffer, find first imu message that is newer than the given
  // timestamp
  ImuAdapter::Iterator imu_it = imu_adapter->begin();
  for (; imu_it != imu_adapter->end(); ++imu_it) {
    if ((*imu_it)->header().timestamp_sec() - timestamp_sec >
        FLAGS_timestamp_sec_tolerance) {
      break;
    }
  }

  if (imu_it != imu_adapter->end()) {  // found one
    if (imu_it == imu_adapter->begin()) {
      AERROR << "IMU queue too short or request too old. "
             << "Oldest timestamp["
             << imu_adapter->GetOldestObserved().header().timestamp_sec()
             << "], Newest timestamp["
             << imu_adapter->GetLatestObserved().header().timestamp_sec()
             << "], timestamp[" << timestamp_sec << "]";
      *imu_msg = imu_adapter->GetOldestObserved();  // the oldest imu
    } else {
      // here is the normal case
      auto imu_it_1 = imu_it;
      imu_it_1--;
      if (!(*imu_it)->has_header() || !(*imu_it_1)->has_header()) {
        AERROR << "imu1 and imu_it_1 must both have header.";
        return false;
      }
      if (!InterpolateIMU(**imu_it_1, **imu_it, timestamp_sec, imu_msg)) {
        AERROR << "failed to interpolate IMU";
        return false;
      }
    }
  } else {
    // give the newest imu, without extrapolation
    *imu_msg = imu_adapter->GetLatestObserved();
    if (imu_msg == nullptr) {
      AERROR << "Fail to get latest observed imu_msg.";
      return false;
    }

    if (!imu_msg->has_header()) {
      AERROR << "imu_msg must have header.";
      return false;
    }
  }
  return true;
}

void LMDLocalization::RunWatchDog() {
  if (!FLAGS_enable_watchdog) return;

  // Add code to implement watch dog
}

}  // namespace localization
}  // namespace apollo
