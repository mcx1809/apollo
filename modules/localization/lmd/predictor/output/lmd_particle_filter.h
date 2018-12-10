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

/**
 * @file lmd_particle_filter.h
 * @brief The class of lmd_particle_filter.
 */

#ifndef MODULES_LOCALIZATION_LMD_PARTICLE_FILTER_H_
#define MODULES_LOCALIZATION_LMD_PARTICLE_FILTER_H_

#include <math.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"
/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

struct Particle {
  int id;
  double x;
  double y;
  double theta;
  double weight;
};

struct LandMarkObs {
  int id;
  std::vector<double> x;
  std::vector<double> y;
};

class Map {
 public:
  struct SingleLandmarks {
    int id_i;
    std::vector<double> x_f;
    std::vector<double> y_f;
  };
  std::vector<SingleLandmarks> landmark_list;
};

class ParticleFilter {
 public:
  ParticleFilter();
  ~ParticleFilter();
  /**
   * @brief Initializes particle filter by initializing particles to Gaussian
   *   distribution around first position and all the weights to 1.
   * @param x Initial x position [m] (simulated estimate from GPS)
   * @param y Initial y position [m]
   * @param theta Initial orientation [rad]
   * @param std[] Array of dimension 3 [standard deviation of x [m], standard
   * deviation of y [m] standard deviation of yaw [rad]]
   */
  void Init(const double x, const double y, const double theta,
            const double std[]);
  /**
   * @brief Predicts the state for the next time step
   *   using the process model.
   * @param delta_t Time between time step t and t+1 in measurements [s]
   * @param std_pos[] Array of dimension 3 [standard deviation of x [m],
   * standard deviation of y [m] standard deviation of yaw [rad]]
   * @param velocity Velocity of car from t to t+1 [m/s]
   * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
   */
  void Prediction(const double delta_t, const double std_pos[],
                  const double velocity, const double yaw_rate);
  /**
   * @brief Finds which observations correspond to which landmarks
   * (likely by using a nearest-neighbors data association).
   * @param predicted Vector of predicted landmark observations
   * @param observations Vector of landmark observations
   */
  void DataAssociation(std::vector<LandMarkObs> predicted,
                       LandMarkObs observations);
  /**
   * @brief Updates the weights for each particle based on the likelihood
   * of the observed measurements.
   * @param sensor_range Range [m] of sensor
   * @param std_landmark[] Array of dimension 2 [standard deviation of range
   * [m], standard deviation of bearing [rad]]
   * @param observations Vector of landmark observations
   * @param map Map class containing map landmarks
   */
  void UpdateWeights(const double sensor_range, const double std_landmark[],
                     const LandMarkObs observations, const Map map_landmarks);
  /**
   * @brief Resamples from the updated set of particles to form
   *   the new set of particles.
   */
  void Resample();
  /**
   * @brief Returns whether particle filter is initialized yet or
   * not.
   */
  const bool Initialized() const;
  std::vector<Particle> particles;
  Map map;

 private:
  /*
   * @brief Computes the Euclidean distance between two 2D points.
   * @param (x1,y1) x and y coordinates of first point
   * @param (x2,y2) x and y coordinates of second point
   * @output Euclidean distance between two 2D points
   */
  double Dist(const double x1, const double y1, const double x2,
              const double y2);

  void CalculateDiff(const Map::SingleLandmarks map_landmark,
                     const LandMarkObs observation, double* x_diff,
                     double* y_diff);

  void ComputeError(const LandMarkObs source, const LandMarkObs ground_truth,
                    double* error);
  /*
   * @brief Read map data struct from a file
   * @param filename given file to read
   * @param map_ptr the ptr point to given data structure to store map data
   */
  bool ReadMapData(const std::string filename, Map* map_ptr);

 private:
  int num_particles_;
  bool is_initialized_;
  std::vector<double> weights_;
};

}  // namespace localization
}  // namespace apollo

#endif /* MODULES_LOCALIZATION_LMD_PARTICLE_FILTER_H_ */
