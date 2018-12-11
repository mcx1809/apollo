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

#include "modules/localization/lmd/predictor/output/lmd_particle_filter.h"

namespace apollo {
namespace localization {
using apollo::common::math::RotateAxis;
namespace {
constexpr double kEPSILON = 1e-4;
}  // namespace

ParticleFilter::ParticleFilter() : num_particles_(0), is_initialized_(false) {}

ParticleFilter::~ParticleFilter() {}

void ParticleFilter::InitParticleFilter(const double x, const double y,
                                        const double theta,
                                        const double std[]) {
  if (!ReadMapData("modules/localization/testdata/map_data.txt", &map)) {
    AERROR << "Could not open map file";
    return;
  }
  num_particles_ = 500;
  std::default_random_engine gen;
  std::normal_distribution<double> xNoise(x, std[0]);
  std::normal_distribution<double> yNoise(y, std[1]);
  std::normal_distribution<double> yawNoise(theta, std[2]);
  for (int i = 0; i < num_particles_; ++i) {
    Particle p = {i, xNoise(gen), yNoise(gen), yawNoise(gen), 1};
    particles.push_back(p);
    weights_.push_back(1);
  }
  is_initialized_ = true;
}

void ParticleFilter::Prediction(const double delta_t, const double std_pos[],
                                const double velocity, const double yaw_rate) {
  std::default_random_engine gen;
  std::normal_distribution<double> xNoise(0, std_pos[0]);
  std::normal_distribution<double> yNoise(0, std_pos[1]);
  std::normal_distribution<double> yawNoise(0, std_pos[2]);
  for (int i = 0; i < num_particles_; ++i) {
    double yaw = 0;
    double delta_x = 0;
    double delta_y = 0;
    double delta_yaw = 0;
    if (fabs(yaw_rate) < kEPSILON) {
      delta_x = velocity * delta_t * cos(yaw);
      delta_y = velocity * delta_t * sin(yaw);
    } else {
      double c = velocity / yaw_rate;
      delta_x = c * (sin(yaw + yaw_rate * delta_t) - sin(yaw));
      delta_y = c * (cos(yaw) - cos(yaw + yaw_rate * delta_t));
      delta_yaw = yaw_rate * delta_t;
    }
    delta_x += xNoise(gen);
    delta_y += yNoise(gen);
    double enu_x, enu_y;
    delta_yaw += yawNoise(gen);
    particles[i].theta += delta_yaw;
    RotateAxis(-particles[i].theta, delta_x, delta_y, &enu_x, &enu_y);
    particles[i].x += enu_x;
    particles[i].y += enu_y;
  }
}

void ParticleFilter::ComputeError(const LandMarkObs source,
                                  const LandMarkObs ground_truth,
                                  double* error) {
  CHECK_EQ(source.x.size(), ground_truth.x.size());
  auto size = source.x.size();
  for (size_t i = 0; i < size; ++i) {
    *error +=
        Dist(source.x[i], source.y[i], ground_truth.x[i], ground_truth.y[i]);
  }
}

void ParticleFilter::CalculateDiff(const Map::SingleLandmarks& map_landmark,
                                   const LandMarkObs& observation,
                                   double* x_diff, double* y_diff) {
  CHECK_NOTNULL(x_diff);
  CHECK_NOTNULL(y_diff);
  double temp_xdiff = 0;
  double temp_ydiff = 0;
  for (size_t i = 0; i < map_landmark.x_f.size(); ++i) {
    temp_xdiff += std::abs(map_landmark.x_f[i] - observation.x[i]);
    temp_ydiff += std::abs(map_landmark.y_f[i] - observation.y[i]);
  }
  *x_diff = temp_xdiff / map_landmark.x_f.size();
  *y_diff = temp_ydiff / map_landmark.y_f.size();
}

void ParticleFilter::DataAssociation(std::vector<LandMarkObs> predicted,
                                     LandMarkObs observations) {
  int min = 1e6;
  for (size_t i = 0; i < predicted.size(); ++i) {
    double temp_error = 0;
    ComputeError(predicted[i], observations, &temp_error);
    if (temp_error < min) {
      min = temp_error;
      observations.id = predicted[i].id;
    }
  }
}

void ParticleFilter::UpdateWeights(const double sensor_range,
                                   const double std_landmark[],
                                   const LandMarkObs observations,
                                   const Map map_landmarks) {
  double weights_sum = 0.0;
  for (auto j = 0; j < num_particles_; ++j) {
    Particle p = particles[j];
    std::vector<LandMarkObs> predicted;
    LandMarkObs gObservations;
    for (size_t i = 0; i < observations.x.size(); ++i) {
      double obs_x = observations.x[i];
      double obs_y = observations.y[i];
      double enu_x, enu_y;
      RotateAxis(-p.theta, obs_x, obs_y, &enu_x, &enu_y);
      gObservations.x.emplace_back(p.x + enu_x);
      gObservations.y.emplace_back(p.y + enu_y);
    }
    for (size_t k = 0; k < map_landmarks.landmark_list.size(); ++k) {
      auto x_lm = map_landmarks.landmark_list[k].x_f;
      auto y_lm = map_landmarks.landmark_list[k].y_f;
      int id_lm = map_landmarks.landmark_list[k].id_i;
      double distance = Dist(x_lm[0], y_lm[0], p.x, p.y);
      if (distance <= sensor_range) {
        LandMarkObs obs = {id_lm, x_lm, y_lm};
        predicted.emplace_back(obs);
      }
    }
    DataAssociation(predicted, gObservations);
    double prob = 1.0;
    double std_x = std_landmark[0];
    double std_y = std_landmark[1];
    double x_diff = 0.0;
    double y_diff = 0.0;
    double c = 1 / (2 * M_PI * std_x * std_y);
    Map::SingleLandmarks lm;
    lm = map_landmarks.landmark_list[gObservations.id];
    CalculateDiff(lm, gObservations, &x_diff, &y_diff);
    prob *= c * exp(-(x_diff + y_diff) / 2);
    weights_[j] = prob;
    particles[j].weight = prob;
    weights_sum += prob;
  }
  if (weights_sum > 0) {
    for (size_t i = 0; i < weights_.size(); ++i) {
      weights_[i] /= weights_sum;
    }
  }
}

void ParticleFilter::Resample() {
  std::default_random_engine gen;
  std::discrete_distribution<int> d(weights_.begin(), weights_.end());
  std::vector<Particle> resampled_particles(particles.size());
  for (int i = 0; i < num_particles_; ++i) {
    int idx = d(gen);
    resampled_particles[i] = particles[idx];
  }
  particles = resampled_particles;
}

const bool ParticleFilter::Initialized() const { return is_initialized_; }

double ParticleFilter::Dist(const double x1, const double y1, const double x2,
                            const double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

bool ParticleFilter::ReadMapData(const std::string filename, Map* map) {
  std::ifstream in_file_map(filename.c_str(), std::ifstream::in);
  if (!in_file_map) {
    AERROR << "Input Map File error";
    return false;
  }
  std::string line_map;
  while (getline(in_file_map, line_map)) {
    std::istringstream iss_map(line_map);
    float landmark_x_f, landmark_y_f;
    int id_i;
    iss_map >> id_i;
    Map::SingleLandmarks single_landmark_temp;
    single_landmark_temp.id_i = id_i;
    for (size_t i = 0; i < 10; ++i) {
      iss_map >> landmark_x_f;
      iss_map >> landmark_y_f;
      single_landmark_temp.x_f.emplace_back(landmark_x_f);
      single_landmark_temp.y_f.emplace_back(landmark_y_f);
    }
    map->landmark_list.emplace_back(single_landmark_temp);
  }
  return true;
}

}  // namespace localization
}  // namespace apollo
