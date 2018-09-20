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
 * @file navi_st_solver.h
 **/

#ifndef MODULES_PLANNING_NAVI_NAVI_ST_SOLVER_H_
#define MODULES_PLANNING_NAVI_NAVI_ST_SOLVER_H_

#include <functional>
#include <limits>
#include <vector>

#include "modules/common/status/status.h"

/**
 * @namespace apollo::planning::navi_st_solver
 * @brief apollo::planning::navi_st_solver
 */
namespace apollo {
namespace planning {
namespace navi_st_solver {

/**
 * @struct VehicleCapability
 * @brief VehicleCapability is used to describe current capability of
 * the vehicle.
 */
struct VehicleCapability {
  // Maximum acceleration.
  double a_max = std::numeric_limits<double>::infinity();
  // Maximum deceleration.
  double b_max = std::numeric_limits<double>::infinity();
  // Maximum jerk.
  double da_max = std::numeric_limits<double>::infinity();
};

/**
 * @struct Constraint
 * @brief Constraint is used to describe constraint.
 */
struct Constraint {
  // Maximum station.
  double s_max = std::numeric_limits<double>::infinity();
  // Preffered station.
  double s_preffered = std::numeric_limits<double>::infinity();
  // Maximum velocity.
  double v_max = std::numeric_limits<double>::infinity();
  // Preffered velocity.
  double v_preffered = std::numeric_limits<double>::infinity();
  // Maximum acceleration.
  double a_max = std::numeric_limits<double>::infinity();
  // Preffered acceleration.
  double a_preffered = std::numeric_limits<double>::infinity();
  // Maximum deceleration.
  double b_max = std::numeric_limits<double>::infinity();
  // Preffered deceleration.
  double b_preffered = std::numeric_limits<double>::infinity();
  // Maximum jerk.
  double da_max = std::numeric_limits<double>::infinity();
  // Preffered jerk.
  double da_preffered = std::numeric_limits<double>::infinity();
};

/**
 * @struct OutputPoint
 * @brief OutputPoint is used to describe a output point.
 */
struct OutputPoint {
  // Timestamp.
  double t;
  // Station.
  double s;
  // Velocity.
  double v;
  // Acceleration.
  double a;
  // Jerk.
  double da;
};

/**
 * @class Solver
 * @brief Solver is used to generate speed planning with constraint.
 */
class Solver {
 public:
  /**
   * @brief Reset solver.
   * @param s_step Resolution of s.
   * @param s_max Max of s.
   * @param t_step Step of t.
   * @param t_max Max of t.
   */
  void Reset(double s_step, double s_max, double t_step, double t_max);

  /**
   * @brief Assign constraint for all.
   * @param con Constraint.
   */
  void UpdateConstraint(const Constraint& con);

  /**
   * @brief Assign constraint for specified time range.
   * @param start_t Start time.
   * @param end_t End time.
   * @param con Constraint.
   */
  void UpdateConstraintForTime(double start_t, double end_t,
                               const Constraint& con);

  /**
   * @brief Assign constraint for specified station range.
   * @param start_s Start station.
   * @param end_s End station.
   * @param con Constraint, the station limits are ignored.
   */
  void UpdateConstraintForStation(double start_s, double end_s,
                                  const Constraint& con);

  /**
   * @brief Solving the s-t curve.
   * @param start_v Velocity of the start point.
   * @param start_a Acceleration of the start point.
   * @param start_da Jerk of the start point.
   * @param get_vehicle_capability Callback for get the vehicle's current
   * capability.
   * @param iter_num The number of iterations.
   * @param resampling The number of resampling for t.
   * @param output Buffer for output.
   * @return Status::OK() if success; error otherwise.
   */
  apollo::common::Status Solve(
      double start_v, double start_a, double start_da,
      const std::function<VehicleCapability(double t, double s, double v,
                                            double a)>& get_vehicle_capability,
      int iter_num, unsigned int resampling, std::vector<OutputPoint>* output);

 private:
  std::vector<Constraint> s_constraint_;
  std::vector<Constraint> t_constraint_;
  double s_max_;
  double s_step_;
  double t_step_;
};

}  // namespace navi_st_solver
}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_NAVI_NAVI_ST_SOLVER_H_
