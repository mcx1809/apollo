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

#include "modules/localization/lmd/lmd_particle_filter.h"

#define EPSILON 1e-4

ParticleFilter::ParticleFilter() : num_particles_(0), is_initialized_(false) {}

ParticleFilter::~ParticleFilter() {}

void ParticleFilter::Init(double x, double y, double theta, double std[]) {
  num_particles_ = 50;
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

void ParticleFilter::Prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  std::default_random_engine gen;
  std::normal_distribution<double> xNoise(0, std_pos[0]);
  std::normal_distribution<double> yNoise(0, std_pos[1]);
  std::normal_distribution<double> yawNoise(0, std_pos[2]);
  for (int i = 0; i < num_particles_; ++i) {
    Particle p = particles[i];
    double yaw = p.theta;
    double delta_x = 0;
    double delta_y = 0;
    double delta_yaw = 0;
    if (fabs(yaw_rate) < EPSILON) {
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
    delta_yaw += yawNoise(gen);
    particles[i].x += delta_x;
    particles[i].y += delta_y;
    particles[i].theta += delta_yaw;
  }
}

void ParticleFilter::DataAssociation(std::vector<LandMarkObs> predicted,
                                     std::vector<LandMarkObs> observations) {
  for (size_t i = 0; i < observations.size(); ++i) {
    int min = 1e6;
    int ld_id = -1;
    double x_obs = observations[i].x;
    double y_obs = observations[i].y;
    for (size_t j = 0; j < predicted.size(); ++j) {
      double x_pred = predicted[j].x;
      double y_pred = predicted[j].y;
      double distance = Dist(x_pred, y_pred, x_obs, y_obs);
      if (distance < min) {
        min = distance;
        ld_id = predicted[j].id;
      }
    }
    observations[i].id = ld_id;
  }
}

void ParticleFilter::UpdateWeights(double sensor_range, double std_landmark[],
                                   std::vector<LandMarkObs> observations,
                                   Map map_landmarks) {
  double weights_sum = 0;
  for (auto j = 0; j < num_particles_; ++j) {
    Particle p = particles[j];
    std::vector<LandMarkObs> predicted;
    std::map<int, int> lm2idx;
    std::vector<LandMarkObs> gObservations(observations.size());
    for (size_t i = 0; i < observations.size(); ++i) {
      double x = observations[i].x;
      double y = observations[i].y;
      gObservations[i].x = x * cos(p.theta) - y * sin(p.theta) + p.x;
      gObservations[i].y = x * sin(p.theta) + y * cos(p.theta) + p.y;
    }
    for (size_t k = 0; k < map_landmarks.landmark_list.size(); ++k) {
      double x_lm = map_landmarks.landmark_list[k].x_f;
      double y_lm = map_landmarks.landmark_list[k].y_f;
      int id_lm = map_landmarks.landmark_list[k].id_i;
      double distance = Dist(x_lm, y_lm, p.x, p.y);
      if (distance <= sensor_range) {
        LandMarkObs obs = {id_lm, x_lm, y_lm};
        predicted.push_back(obs);
        lm2idx[id_lm] = k;
      }
    }
    DataAssociation(predicted, gObservations);
    double prob = 1.0;
    double std_x = std_landmark[0];
    double std_y = std_landmark[1];
    double c = 1 / (2 * M_PI * std_x * std_y);
    for (size_t m = 0; m < gObservations.size(); ++m) {
      LandMarkObs obs = gObservations[m];
      Map::SingleLandmarks lm;
      int landmark_id = obs.id;
      lm = map_landmarks.landmark_list[lm2idx[landmark_id]];
      double x_obs = obs.x;
      double y_obs = obs.y;
      double x_lm = lm.x_f;
      double y_lm = lm.y_f;
      double x_diff = pow((x_obs - x_lm) / std_x, 2.0);
      double y_diff = pow((y_obs - y_lm) / std_y, 2.0);
      prob *= c * exp(-(x_diff + y_diff) / 2);
    }
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

double ParticleFilter::Dist(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

bool ParticleFilter::ReadMapData(std::string filename, Map map) {
  std::ifstream in_file_map(filename.c_str(), std::ifstream::in);
  if (!in_file_map) {
    return false;
  }
  std::string line_map;
  while (getline(in_file_map, line_map)) {
    std::istringstream iss_map(line_map);
    float landmark_x_f, landmark_y_f;
    int id_i;
    iss_map >> landmark_x_f;
    iss_map >> landmark_y_f;
    iss_map >> id_i;
    Map::SingleLandmarks single_landmark_temp;
    single_landmark_temp.id_i = id_i;
    single_landmark_temp.x_f = landmark_x_f;
    single_landmark_temp.y_f = landmark_y_f;
    map.landmark_list.push_back(single_landmark_temp);
  }
  return true;
}
