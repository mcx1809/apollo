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

#include "modules/localization/lmd/predictor/output/predictor_output.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "modules/common/proto/geometry.pb.h"

#include "modules/common/log.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using apollo::common::Point3D;
using apollo::common::PointENU;
using apollo::common::Quaternion;
using apollo::common::Status;
using apollo::common::math::EulerAnglesZXYd;
using apollo::common::math::HeadingToQuaternion;
using apollo::common::math::NormalizeAngle;
using apollo::common::math::QuaternionRotate;
using apollo::common::math::QuaternionToHeading;
using apollo::common::math::RotateAxis;

namespace {
constexpr double kSamplingInterval = 0.01;
}  // namespace

namespace {
template <class T>
T QuaternionRotateXYZ(const T v, const Quaternion& orientation) {
  auto vec =
      QuaternionRotate(orientation, Eigen::Vector3d(v.x(), v.y(), v.z()));
  T ret;
  ret.set_x(vec[0]);
  ret.set_y(vec[1]);
  ret.set_z(vec[2]);
  return ret;
}

void FillPoseFromImu(const Pose& imu_pose, Pose* pose) {
  // linear acceleration
  if (imu_pose.has_linear_acceleration()) {
    if (FLAGS_enable_map_reference_unify) {
      if (pose->has_orientation()) {
        pose->mutable_linear_acceleration()->CopyFrom(QuaternionRotateXYZ(
            imu_pose.linear_acceleration(), pose->orientation()));
        pose->mutable_linear_acceleration_vrf()->CopyFrom(
            imu_pose.linear_acceleration());
      } else {
        AERROR << "Fail to convert linear_acceleration";
      }
    } else {
      pose->mutable_linear_acceleration()->CopyFrom(
          imu_pose.linear_acceleration());
    }
  }

  // angular velocity
  if (imu_pose.has_angular_velocity()) {
    if (FLAGS_enable_map_reference_unify) {
      if (FLAGS_enable_map_reference_unify) {
        if (pose->has_orientation()) {
          pose->mutable_angular_velocity()->CopyFrom(QuaternionRotateXYZ(
              imu_pose.angular_velocity(), pose->orientation()));
          pose->mutable_angular_velocity_vrf()->CopyFrom(
              imu_pose.angular_velocity());
        } else {
          AERROR << "Fail to convert angular_velocity";
        }
      } else {
        pose->mutable_angular_velocity()->CopyFrom(imu_pose.angular_velocity());
      }
    } else {
      pose->mutable_angular_velocity()->CopyFrom(imu_pose.angular_velocity());
    }
  }

  // euler angle
  if (imu_pose.has_euler_angles()) {
    pose->mutable_euler_angles()->CopyFrom(imu_pose.euler_angles());
  }
}

const std::string& PredictorGpsName() {
  static const std::string name(kPredictorGpsName);
  return name;
}

const std::string& PredictorImuName() {
  static const std::string name(kPredictorFilteredImuName);
  return name;
}

const std::string& PredictorPerceptionName() {
  static const std::string name(kPredictorPerceptionName);
  return name;
}
}  // namespace

PredictorOutput::PredictorOutput(
    double memory_cycle_sec,
    const std::function<Status(double, const Pose&)>& publish_loc_func)
    : Predictor(memory_cycle_sec), publish_loc_func_(publish_loc_func) {
  name_ = kPredictorOutputName;
  dep_predicteds_.emplace(PredictorGpsName(), PoseList(memory_cycle_sec));
  dep_predicteds_.emplace(PredictorImuName(), PoseList(memory_cycle_sec));
  dep_predicteds_.emplace(PredictorPerceptionName(),
                          PoseList(memory_cycle_sec));
  on_adapter_thread_ = true;

  constexpr double kLCutoffFreq = 20.0;
  InitSecondOrderLPFilter(kLCutoffFreq);
  is_initfilterparam_ = false;
}

PredictorOutput::~PredictorOutput() {}

bool PredictorOutput::Updateable() const {
  const auto& imu = dep_predicteds_.find(PredictorImuName())->second;
  if (predicted_.empty()) {
    const auto& gps = dep_predicteds_.find(PredictorGpsName())->second;
    return !gps.empty() && !imu.empty();
  } else {
    return !imu.empty() && predicted_.Older(imu);
  }
}

Status PredictorOutput::Update() {
  if (predicted_.empty()) {
    const auto& gps = dep_predicteds_[PredictorGpsName()];
    auto gps_latest = gps.Latest();
    auto timestamp_sec = gps_latest->first;
    auto pose = gps_latest->second;

    if (!pose.has_heading() && pose.has_orientation()) {
      pose.set_heading(QuaternionToHeading(
          pose.orientation().qw(), pose.orientation().qx(),
          pose.orientation().qy(), pose.orientation().qz()));
    }

    const auto& imu = dep_predicteds_[PredictorImuName()];
    Pose imu_pose;
    imu.FindNearestPose(timestamp_sec, &imu_pose);

    // fill pose from imu
    FillPoseFromImu(imu_pose, &pose);

    // push pose to list
    predicted_.Push(timestamp_sec, pose);

    // publish
    return publish_loc_func_(timestamp_sec, pose);
  } else {
    // get timestamp from imu
    const auto& imu = dep_predicteds_[PredictorImuName()];
    auto timestamp_sec = imu.Latest()->first;

    // base pose for prediction
    double base_timestamp_sec;
    Pose base_pose;
    const auto& perception = dep_predicteds_[PredictorPerceptionName()];
    auto perception_pose_it = perception.RangeOf(timestamp_sec).first;
    if (perception_pose_it != perception.end()) {
      base_timestamp_sec = perception_pose_it->first;
      if (!predicted_.Older(base_timestamp_sec)) {
        predicted_.FindNearestPose(base_timestamp_sec, &base_pose);
        // assign position and heading from perception
        const auto& perception_pose = perception_pose_it->second;
        base_pose.mutable_position()->CopyFrom(perception_pose.position());
        base_pose.mutable_orientation()->CopyFrom(
            perception_pose.orientation());
        base_pose.set_heading(perception_pose.heading());
      } else {
        base_timestamp_sec = predicted_.Latest()->first;
        base_pose = predicted_.Latest()->second;
      }
    } else {
      base_timestamp_sec = predicted_.Latest()->first;
      base_pose = predicted_.Latest()->second;
    }

    // predict
    Pose pose;
    PredictByImu(base_timestamp_sec, base_pose, timestamp_sec, &pose);

    // push pose to list
    predicted_.Push(timestamp_sec, pose);

    // publish
    return publish_loc_func_(timestamp_sec, pose);
  }
}

bool PredictorOutput::UpdateLaneMarkers(
    double timestamp_sec, const apollo::perception::LaneMarkers& lane_markers) {
  if (!lane_markers.has_left_lane_marker()) {
    AERROR << "There are no left lane marker in perception obstacles";
    return false;
  }
  const auto& lanemarker = lane_markers.left_lane_marker();
  if (!lanemarker.has_c0_position() || !lanemarker.has_c1_heading_angle() ||
      !lanemarker.has_c2_curvature() ||
      !lanemarker.has_c3_curvature_derivative()) {
    AERROR << "left lane marker has no required params";
    return false;
  }
  lane_markers_.CopyFrom(lane_markers);
  lane_markers_time_ = timestamp_sec;
  return true;
}

bool PredictorOutput::PredictByParticleFilter(double old_timestamp_sec,
                                              const Pose& old_pose,
                                              double new_timestamp_sec,
                                              Pose* new_pose) {
  if (!old_pose.has_position() || !old_pose.has_orientation() ||
      !old_pose.has_linear_velocity()) {
    AERROR << "Old_pose has no some fields";
    return false;
  }
  new_pose->CopyFrom(old_pose);
  double sensor_range = 5;
  double sigma_pos[3] = {0.03, 0.03, 0.001};
  double sigma_landmark[2] = {0.03, 0.03};
  std::default_random_engine gen;
  std::normal_distribution<double> N_x_init(0, sigma_pos[0]);
  std::normal_distribution<double> N_y_init(0, sigma_pos[1]);
  std::normal_distribution<double> N_theta_init(0, sigma_pos[2]);
  std::normal_distribution<double> N_obs_x(0, sigma_landmark[0]);
  std::normal_distribution<double> N_obs_y(0, sigma_landmark[1]);
  auto delta_time = new_timestamp_sec - old_timestamp_sec;
  if (!pc_filter_.Initialized()) {
    double n_x, n_y, n_theta;
    auto x = new_pose->position().x();
    auto y = new_pose->position().y();
    auto theta = new_pose->heading();
    n_x = N_x_init(gen);
    n_y = N_y_init(gen);
    n_theta = N_theta_init(gen);
    pc_filter_.Init(x + n_x, y + n_y, theta + n_theta, sigma_pos);
  } else {
    auto velocity_x = new_pose->linear_velocity().x();
    auto velocity_y = new_pose->linear_velocity().y();
    auto velocity_z = new_pose->linear_velocity().z();
    auto velocity = sqrt(pow(velocity_x, 2.0) + pow(velocity_y, 2.0) +
                         pow(velocity_z, 2.0));
    auto yawrate = new_pose->angular_velocity().z();
    pc_filter_.Prediction(delta_time, sigma_pos, velocity, yawrate);
  }
  const auto& lanemarker = lane_markers_.left_lane_marker();
  auto c0 = lanemarker.c0_position();
  auto c1 = lanemarker.c1_heading_angle();
  auto c2 = lanemarker.c2_curvature();
  auto c3 = lanemarker.c3_curvature_derivative();

  LandMarkObs noisy_observations;
  double n_x, n_y;
  for (int j = 0; j < 10; ++j) {
    n_x = N_obs_x(gen);
    n_y = N_obs_y(gen);
    auto x = 0.1 * j + n_x;
    auto y = c3 * pow(x, 3.0) + c2 * pow(x, 2.0) + c1 * x + c0 + n_y;
    noisy_observations.x.emplace_back(x);
    noisy_observations.y.emplace_back(y);
  }
  pc_filter_.UpdateWeights(sensor_range, sigma_landmark, noisy_observations,
                           pc_filter_.map);
  pc_filter_.Resample();
  std::vector<Particle> particles = pc_filter_.particles;
  double highest_weight = -1.0;
  auto best_particle = particles[0];
  for (std::size_t i = 0; i < particles.size(); ++i) {
    if (particles[i].weight > highest_weight) {
      highest_weight = particles[i].weight;
      best_particle = particles[i];
    }
  }
  PointENU position_1;
  position_1.set_x(best_particle.x);
  position_1.set_y(best_particle.y);
  position_1.set_z(old_pose.position().z());
  new_pose->mutable_position()->CopyFrom(position_1);
  const auto& imu = dep_predicteds_[PredictorImuName()];
  auto p = imu.RangeOf(old_timestamp_sec);
  auto it = p.first;
  if (it == imu.end()) {
    if (p.second != imu.end()) {
      it = p.second;
    } else {
      AERROR << std::setprecision(15)
             << "Cannot get the lower of range from imu with timestamp["
             << old_timestamp_sec << "]";
      return false;
    }
  }
  auto timestamp_sec = old_timestamp_sec;
  bool finished = false;
  while (!finished) {
    Pose imu_pose;
    double timestamp_sec_1;
    Pose imu_pose_1;
    auto it_1 = it;
    it_1++;
    if (it_1 != imu.end() && new_timestamp_sec >= it_1->first) {
      PoseList::InterpolatePose(it->first, it->second, it_1->first,
                                it_1->second, timestamp_sec, &imu_pose);
      timestamp_sec_1 = it_1->first;
      imu_pose_1 = it_1->second;
    } else {
      timestamp_sec_1 = new_timestamp_sec;
      imu_pose = it->second;
      imu_pose_1 = imu_pose;
    }
    if (new_timestamp_sec <= timestamp_sec_1) {
      finished = true;
    }
    if (!finished && timestamp_sec_1 <= timestamp_sec) {
      it = it_1;
      imu_pose = imu_pose_1;
      continue;
    }
    if (!imu_pose.has_linear_acceleration() ||
        !imu_pose_1.has_linear_acceleration() ||
        !imu_pose.has_angular_velocity() ||
        !imu_pose_1.has_angular_velocity()) {
      AERROR << "Imu_pose or imu_pose_1 has no some fields";
      return false;
    }
    auto dt = timestamp_sec_1 - timestamp_sec;
    auto orientation = new_pose->orientation();
    Point3D angular_velocity;
    if (FLAGS_enable_map_reference_unify) {
      angular_velocity.CopyFrom(
          QuaternionRotateXYZ(imu_pose.angular_velocity(), orientation));
    } else {
      angular_velocity.CopyFrom(imu_pose.angular_velocity());
    }
    Point3D angular_velocity_1;
    if (FLAGS_enable_map_reference_unify) {
      angular_velocity_1.CopyFrom(
          QuaternionRotateXYZ(imu_pose_1.angular_velocity(), orientation));
    } else {
      angular_velocity_1.CopyFrom(imu_pose_1.angular_velocity());
    }
    Point3D angular_vel;
    angular_vel.set_x((angular_velocity.x() + angular_velocity_1.x()) / 2.0);
    angular_vel.set_y((angular_velocity.y() + angular_velocity_1.y()) / 2.0);
    angular_vel.set_z((angular_velocity.z() + angular_velocity_1.z()) / 2.0);
    EulerAnglesZXYd euler_a(orientation.qw(), orientation.qx(),
                            orientation.qy(), orientation.qz());
    auto derivation_roll =
        angular_vel.x() +
        std::sin(euler_a.roll()) * std::tan(euler_a.pitch()) * angular_vel.y() +
        std::cos(euler_a.roll()) * std::tan(euler_a.pitch()) * angular_vel.z();
    auto derivation_pitch = std::cos(euler_a.roll()) * angular_vel.y() -
                            std::sin(euler_a.roll()) * angular_vel.z();
    auto derivation_yaw =
        std::sin(euler_a.roll()) / std::cos(euler_a.pitch()) * angular_vel.y() +
        std::cos(euler_a.roll()) / std::cos(euler_a.pitch()) * angular_vel.z();
    EulerAnglesZXYd euler_b(euler_a.roll() + derivation_roll * dt,
                            euler_a.pitch() + derivation_pitch * dt,
                            euler_a.yaw() + derivation_yaw * dt);
    auto q = euler_b.ToQuaternion();
    Quaternion orientation_1;
    orientation_1.set_qw(q.w());
    orientation_1.set_qx(q.x());
    orientation_1.set_qy(q.y());
    orientation_1.set_qz(q.z());
    Point3D linear_acceleration;
    if (FLAGS_enable_map_reference_unify) {
      linear_acceleration.CopyFrom(
          QuaternionRotateXYZ(imu_pose.linear_acceleration(), orientation));
    } else {
      linear_acceleration.CopyFrom(imu_pose.linear_acceleration());
    }
    Point3D linear_acceleration_1;
    if (FLAGS_enable_map_reference_unify) {
      linear_acceleration_1.CopyFrom(
          QuaternionRotateXYZ(imu_pose_1.linear_acceleration(), orientation_1));
    } else {
      linear_acceleration_1.CopyFrom(imu_pose_1.linear_acceleration());
    }
    auto linear_velocity = new_pose->linear_velocity();
    Point3D linear_velocity_1;
    linear_velocity_1.set_x(
        (linear_acceleration.x() + linear_acceleration_1.x()) / 2.0 * dt +
        linear_velocity.x());
    linear_velocity_1.set_y(
        (linear_acceleration.y() + linear_acceleration_1.y()) / 2.0 * dt +
        linear_velocity.y());
    linear_velocity_1.set_z(
        (linear_acceleration.z() + linear_acceleration_1.z()) / 2.0 * dt +
        linear_velocity.z());
    new_pose->mutable_orientation()->CopyFrom(orientation_1);
    new_pose->set_heading(QuaternionToHeading(
        new_pose->orientation().qw(), new_pose->orientation().qx(),
        new_pose->orientation().qy(), new_pose->orientation().qz()));
    new_pose->mutable_linear_velocity()->CopyFrom(linear_velocity_1);
    FillPoseFromImu(imu_pose_1, new_pose);
    if (!finished) {
      timestamp_sec = timestamp_sec_1;
      it = it_1;
      imu_pose = imu_pose_1;
    }
  }
  return true;
}

bool PredictorOutput::PredictByImu(double old_timestamp_sec,
                                   const Pose& old_pose,
                                   double new_timestamp_sec, Pose* new_pose) {
  if (!old_pose.has_position() || !old_pose.has_orientation() ||
      !old_pose.has_linear_velocity()) {
    AERROR << "Pose has no some fields";
    return false;
  }

  const auto& imu = dep_predicteds_[PredictorImuName()];
  auto p = imu.RangeOf(old_timestamp_sec);
  auto it = p.first;
  auto it_1 = p.second;
  if (it == imu.end() && it_1 == imu.end()) {
    AERROR << std::setprecision(15)
           << "Cannot get the lower of range from imu with timestamp["
           << old_timestamp_sec << "]";
    return false;
  }

  auto timestamp_sec = old_timestamp_sec;
  new_pose->CopyFrom(old_pose);
  bool finished = false;
  while (!finished) {
    Pose imu_pose;
    double timestamp_sec_1;
    Pose imu_pose_1;

    if (it == imu.end()) {
      imu_pose = imu_pose_1 = it_1->second;
      timestamp_sec_1 = std::min(it_1->first, new_timestamp_sec);
    } else if (it_1 == imu.end()) {
      imu_pose = imu_pose_1 = it->second;
      timestamp_sec_1 = new_timestamp_sec;
    } else {
      PoseList::InterpolatePose(it->first, it->second, it_1->first,
                                it_1->second, timestamp_sec, &imu_pose);
      timestamp_sec_1 = std::min(it_1->first, new_timestamp_sec);
      PoseList::InterpolatePose(it->first, it->second, it_1->first,
                                it_1->second, timestamp_sec_1, &imu_pose_1);
    }

    if (new_timestamp_sec <= timestamp_sec_1) {
      finished = true;
    }

    if (!finished && timestamp_sec_1 <= timestamp_sec) {
      it = it_1;
      it_1++;
      continue;
    }

    if (!imu_pose.has_linear_acceleration() ||
        !imu_pose_1.has_linear_acceleration() ||
        !imu_pose.has_angular_velocity() ||
        !imu_pose_1.has_angular_velocity()) {
      AERROR << "Imu_pose or imu_pose_1 has no some fields";
      return false;
    }

    auto dt = timestamp_sec_1 - timestamp_sec;
    auto orientation = new_pose->orientation();

    Point3D angular_velocity;
    if (FLAGS_enable_map_reference_unify) {
      angular_velocity.CopyFrom(
          QuaternionRotateXYZ(imu_pose.angular_velocity(), orientation));
    } else {
      angular_velocity.CopyFrom(imu_pose.angular_velocity());
    }
    Point3D angular_velocity_1;
    if (FLAGS_enable_map_reference_unify) {
      angular_velocity_1.CopyFrom(
          QuaternionRotateXYZ(imu_pose_1.angular_velocity(), orientation));
    } else {
      angular_velocity_1.CopyFrom(imu_pose_1.angular_velocity());
    }

    Point3D angular_vel;
    angular_vel.set_x((angular_velocity.x() + angular_velocity_1.x()) / 2.0);
    angular_vel.set_y((angular_velocity.y() + angular_velocity_1.y()) / 2.0);
    angular_vel.set_z((angular_velocity.z() + angular_velocity_1.z()) / 2.0);

    EulerAnglesZXYd euler_c;
    if (FLAGS_enable_gps_heading) {
      const auto& gps = dep_predicteds_[PredictorGpsName()];
      auto gps_latest = gps.Latest();
      auto gps_pose = gps_latest->second;

      if (!gps_pose.has_orientation()) {
        gps_pose.CopyFrom(old_pose);
      }
      EulerAnglesZXYd euler_b(
          gps_pose.orientation().qw(), gps_pose.orientation().qx(),
          gps_pose.orientation().qy(), gps_pose.orientation().qz());
      euler_c = euler_b;

    } else {
      EulerAnglesZXYd euler_a(orientation.qw(), orientation.qx(),
                              orientation.qy(), orientation.qz());

      auto derivation_roll = angular_vel.x() +
                             std::sin(euler_a.roll()) *
                                 std::tan(euler_a.pitch()) * angular_vel.y() +
                             std::cos(euler_a.roll()) *
                                 std::tan(euler_a.pitch()) * angular_vel.z();

      auto derivation_pitch = std::cos(euler_a.roll()) * angular_vel.y() -
                              std::sin(euler_a.roll()) * angular_vel.z();

      auto derivation_yaw = std::sin(euler_a.roll()) /
                                std::cos(euler_a.pitch()) * angular_vel.y() +
                            std::cos(euler_a.roll()) /
                                std::cos(euler_a.pitch()) * angular_vel.z();

      EulerAnglesZXYd euler_b(euler_a.roll() + derivation_roll * dt,
                              euler_a.pitch() + derivation_pitch * dt,
                              euler_a.yaw() + derivation_yaw * dt);

      euler_c = euler_b;
    }

    auto q = euler_c.ToQuaternion();
    Quaternion orientation_1;
    orientation_1.set_qw(q.w());
    orientation_1.set_qx(q.x());
    orientation_1.set_qy(q.y());
    orientation_1.set_qz(q.z());

    if (FLAGS_enable_heading_filter) {
      auto heading = NormalizeAngle(
          QuaternionToHeading(orientation_1.qw(), orientation_1.qx(),
                              orientation_1.qy(), orientation_1.qz()));

      auto filtered_heading = NormalizeAngle(SecondOrderLPFilter(heading));

      auto orientation_2 = HeadingToQuaternion(filtered_heading);
      orientation_1.set_qw(orientation_2.w());
      orientation_1.set_qx(orientation_2.x());
      orientation_1.set_qy(orientation_2.y());
      orientation_1.set_qz(orientation_2.z());
    }

    Point3D linear_acceleration;
    if (FLAGS_enable_map_reference_unify) {
      linear_acceleration.CopyFrom(
          QuaternionRotateXYZ(imu_pose.linear_acceleration(), orientation));
    } else {
      linear_acceleration.CopyFrom(imu_pose.linear_acceleration());
    }
    Point3D linear_acceleration_1;
    if (FLAGS_enable_map_reference_unify) {
      linear_acceleration_1.CopyFrom(
          QuaternionRotateXYZ(imu_pose_1.linear_acceleration(), orientation_1));
    } else {
      linear_acceleration_1.CopyFrom(imu_pose_1.linear_acceleration());
    }

    auto linear_velocity = new_pose->linear_velocity();
    Point3D linear_velocity_1;
    linear_velocity_1.set_x(
        (linear_acceleration.x() + linear_acceleration_1.x()) / 2.0 * dt +
        linear_velocity.x());
    linear_velocity_1.set_y(
        (linear_acceleration.y() + linear_acceleration_1.y()) / 2.0 * dt +
        linear_velocity.y());
    linear_velocity_1.set_z(
        (linear_acceleration.z() + linear_acceleration_1.z()) / 2.0 * dt +
        linear_velocity.z());

    auto position = new_pose->position();
    PointENU position_1;
    position_1.set_x(
        (linear_acceleration.x() / 3.0 + linear_acceleration_1.x() / 6.0) * dt *
            dt +
        linear_velocity.x() * dt + position.x());
    position_1.set_y(
        (linear_acceleration.y() / 3.0 + linear_acceleration_1.y() / 6.0) * dt *
            dt +
        linear_velocity.y() * dt + position.y());
    position_1.set_z(
        (linear_acceleration.z() / 3.0 + linear_acceleration_1.z() / 6.0) * dt *
            dt +
        linear_velocity.z() * dt + position.z());

    new_pose->mutable_position()->CopyFrom(position_1);
    new_pose->mutable_orientation()->CopyFrom(orientation_1);

    new_pose->set_heading(NormalizeAngle(QuaternionToHeading(
        new_pose->orientation().qw(), new_pose->orientation().qx(),
        new_pose->orientation().qy(), new_pose->orientation().qz())));

    new_pose->mutable_linear_velocity()->CopyFrom(linear_velocity_1);
    FillPoseFromImu(imu_pose_1, new_pose);

    if (!finished) {
      timestamp_sec = timestamp_sec_1;
      it = it_1;
      it_1++;
    }
  }

  return true;
}
bool PredictorOutput::InitSecondOrderLPFilter(double cutoff_freq) {
  auto omega = 2.0 * M_PI * cutoff_freq * kSamplingInterval;
  auto sin_o = std::sin(omega);
  auto cos_o = std::cos(omega);
  auto alpha = sin_o / (2.0 / std::sqrt(2));
  iir_filter_bz_[0] = (1.0 - cos_o) / 2.0;
  iir_filter_bz_[1] = 1.0 - cos_o;
  iir_filter_bz_[2] = (1.0 - cos_o) / 2.0;
  iir_filter_az_[0] = 1.0 + alpha;
  iir_filter_az_[1] = -2.0 * cos_o;
  iir_filter_az_[2] = 1.0 - alpha;
  return true;
}

double PredictorOutput::SecondOrderLPFilter(const double cur_valuer) {
  //  two-order Butterworth filter
  // Y（n)= (b0xn + b1xn-1 + b2xn-2 - (a1xn-1 + a2*xn-2))/a0;
  if (!is_initfilterparam_) {
    iir_filter_val_[0] = cur_valuer;
    iir_filter_val_[1] = cur_valuer;
    iir_filter_val_[2] = cur_valuer;
    is_initfilterparam_ = true;
  } else {
    iir_filter_val_[0] = iir_filter_val_[1];
    iir_filter_val_[1] = iir_filter_val_[2];
    iir_filter_val_[2] = cur_valuer;
  }

  return (iir_filter_bz_[0] * iir_filter_val_[2] +
          iir_filter_bz_[1] * iir_filter_val_[1] +
          iir_filter_bz_[2] * iir_filter_val_[0] -
          (iir_filter_az_[1] * iir_filter_val_[1] +
           iir_filter_az_[2] * iir_filter_val_[0])) /
         iir_filter_az_[0];
}

}  // namespace localization
}  // namespace apollo
