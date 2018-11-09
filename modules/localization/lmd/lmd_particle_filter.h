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
#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <math.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

struct Particle {
  int id;
  double x;
  double y;
  double theta;
  double weight;
};

struct ControlS {
  double velocity;  // Velocity [m/s]
  double yawrate;   // Yaw rate [rad/s]
};

struct GroundTruth {
  double x;      // Global vehicle x position
  double y;      // Global vehicle y position
  double theta;  // Global vehicle yaw [rad]
};

struct LandMarkObs {
  int id;
  double x;
  double y;
};

class Map {
 public:
  struct SingleLandmarks {
    int id_i;
    float x_f;
    float y_f;
  };
  std::vector<SingleLandmarks> landmark_list;
};

class ParticleFilter {
  int num_particles_;
  bool is_initialized_;
  std::vector<double> weights_;

 public:
  std::vector<Particle> particles;
  ParticleFilter();
  ~ParticleFilter();

  /**
   * init Initializes particle filter by initializing particles to Gaussian
   *   distribution around first position and all the weights to 1.
   * @param x Initial x position [m] (simulated estimate from GPS)
   * @param y Initial y position [m]
   * @param theta Initial orientation [rad]
   * @param std[] Array of dimension 3 [standard deviation of x [m], standard
   * deviation of y [m] standard deviation of yaw [rad]]
   */
  void Init(double x, double y, double theta, double std[]);

  /**
   * prediction Predicts the state for the next time step
   *   using the process model.
   * @param delta_t Time between time step t and t+1 in measurements [s]
   * @param std_pos[] Array of dimension 3 [standard deviation of x [m],
   * standard deviation of y [m] standard deviation of yaw [rad]]
   * @param velocity Velocity of car from t to t+1 [m/s]
   * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
   */
  void Prediction(double delta_t, double std_pos[], double velocity,
                  double yaw_rate);

  /**
   * dataAssociation Finds which observations correspond to which landmarks
   * (likely by using a nearest-neighbors data association).
   * @param predicted Vector of predicted landmark observations
   * @param observations Vector of landmark observations
   */
  void DataAssociation(std::vector<LandMarkObs> predicted,
                       std::vector<LandMarkObs> observations);

  /**
   * updateWeights Updates the weights for each particle based on the likelihood
   * of the observed measurements.
   * @param sensor_range Range [m] of sensor
   * @param std_landmark[] Array of dimension 2 [standard deviation of range
   * [m], standard deviation of bearing [rad]]
   * @param observations Vector of landmark observations
   * @param map Map class containing map landmarks
   */
  void UpdateWeights(double sensor_range, double std_landmark[],
                     std::vector<LandMarkObs> observations, Map map_landmarks);

  /**
   * resample Resamples from the updated set of particles to form
   *   the new set of particles.
   */
  void Resample();

  /**
   * initialized Returns whether particle filter is initialized yet or not.
   */
  const bool Initialized() const;
  /*
   * Computes the Euclidean distance between two 2D points.
   * @param (x1,y1) x and y coordinates of first point
   * @param (x2,y2) x and y coordinates of second point
   * @output Euclidean distance between two 2D points
   */
  double Dist(double x1, double y1, double x2, double y2);

  bool ReadMapData(std::string filename, Map map);
};
#endif /* PARTICLE_FILTER_H_ */
