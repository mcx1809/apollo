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
 * @file navi_st_solver.cc
 **/

#include "modules/planning/navi/decider/navi_st_solver.h"

#include <algorithm>
#include <cmath>
//
#include <iostream>

#include "glog/logging.h"

#include "modules/common/log.h"

namespace apollo {
namespace planning {
namespace navi_st_solver {

using apollo::common::ErrorCode;
using apollo::common::Status;

namespace {
// TODO(all):
constexpr double kCostThreshold = 1000.0;
constexpr double kCostSMaxRatio = 10.0;
constexpr double kCostSPrefferedRatio = 1.0;
constexpr double kCostVMaxRatio = 4.0;
constexpr double kCostVPrefferedRatio = 1.0;
constexpr double kCostAMaxRatio = 4.0;
constexpr double kCostAPrefferedRatio = 4.0;
constexpr double kCostJerkMaxRatio = 4.0;
constexpr double kCostJerkPrefferedRatio = 1.0;
constexpr int kFindCycleTimes = 6;
}  // namespace

static void CheckConstraint(const Constraint& con) {
  CHECK_GE(con.s_preffered, 0.0);
  CHECK_GE(con.s_max, con.s_preffered);
  CHECK_GE(con.v_preffered, 0.0);
  CHECK_GE(con.v_max, con.v_preffered);
  CHECK_GE(con.a_preffered, 0.0);
  CHECK_GE(con.a_max, con.a_preffered);
  CHECK_GE(con.b_preffered, 0.0);
  CHECK_GE(con.b_max, con.b_preffered);
  CHECK_GE(con.da_preffered, 0.0);
  CHECK_GE(con.da_max, con.da_preffered);
}

static void CombineConstraint(const Constraint& con, Constraint* dst) {
  dst->s_max = std::min(con.s_max, dst->s_max);
  dst->s_preffered = std::min(con.s_preffered, dst->s_preffered);
  dst->v_max = std::min(con.v_max, dst->v_max);
  dst->v_preffered = std::min(con.v_preffered, dst->v_preffered);
  dst->a_max = std::min(con.a_max, dst->a_max);
  dst->a_preffered = std::min(con.a_preffered, dst->a_preffered);
  dst->da_max = std::min(con.da_max, dst->da_max);
  dst->da_preffered = std::min(con.da_preffered, dst->da_preffered);
}

void Solver::Reset(double s_step, double s_max, double t_step, double t_max) {
  CHECK_GT(s_step, 0.0);
  CHECK_GE(s_max, s_step);
  CHECK_GT(t_step, 0.0);
  CHECK_GE(t_max, t_step);

  s_max_ = s_max;
  s_step_ = s_step;
  t_step_ = t_step;

  auto s_point_num = (std::size_t)((s_max_ + s_step_) / s_step_);
  s_constraint_.clear();
  s_constraint_.resize(s_point_num);

  auto t_point_num = (std::size_t)((t_max + t_step_) / t_step_);
  t_constraint_.clear();
  t_constraint_.resize(t_point_num);
}

void Solver::UpdateConstraint(const Constraint& con) {
  CheckConstraint(con);

  for (auto& dst : t_constraint_) CombineConstraint(con, &dst);
}

void Solver::UpdateConstraintForTime(double start_t, double end_t,
                                     const Constraint& con) {
  CHECK_GE(start_t, 0.0);
  CHECK_GE(end_t, start_t);
  CheckConstraint(con);

  auto start_i = (std::size_t)(std::floor(start_t / t_step_));
  auto end_i = (std::size_t)(std::ceil(end_t / t_step_));
  if (start_i == end_i) {
    CombineConstraint(con, &t_constraint_[start_i]);
  } else {
    for (std::size_t i = start_i; i < end_i && i < t_constraint_.size(); ++i)
      CombineConstraint(con, &t_constraint_[i]);
  }
}

void Solver::UpdateConstraintForStation(double start_s, double end_s,
                                        const Constraint& con) {
  CHECK_GE(start_s, 0.0);
  CHECK_GE(end_s, start_s);
  CheckConstraint(con);

  auto con1 = con;
  con1.s_max = std::numeric_limits<double>::max();
  con1.s_preffered = std::numeric_limits<double>::max();

  auto start_i = (std::size_t)(std::floor(start_s / s_step_));
  auto end_i = (std::size_t)(std::ceil(end_s / s_step_));
  if (start_i == end_i) {
    CombineConstraint(con1, &s_constraint_[start_i]);
  } else {
    for (size_t i = start_i; i < end_i && i < s_constraint_.size(); ++i)
      CombineConstraint(con1, &s_constraint_[i]);
  }
}

//
#define TEST_P ((p.t > 39.0 && p.t < 41.0) && true)
#define TEST_P1 ((cur.t > 39.0 && cur.t < 41.0) && true)
Status Solver::Solve(
    double start_v, double start_a, double start_da,
    const std::function<VehicleCapability(double, double, double, double)>&
        get_vehicle_capability,
    int iter_num, unsigned int resampling, std::vector<OutputPoint>* output) {
  CHECK_NOTNULL(output);

  if (t_constraint_.size() < 5) {
    AERROR << "t_max is too small.";
    return Status(ErrorCode::PLANNING_ERROR, "t_max is too small.");
  }

  // s-t curve
  struct StPoint {
    double t;
    double s;
  };
  std::vector<StPoint> st_curve;
  st_curve.resize(2 + t_constraint_.size() + 3);
  auto process_buf = &st_curve[3];
  auto process_len = st_curve.size() - 3 - 3;

  // make t
  for (std::size_t i = 0; i < st_curve.size(); ++i)
    st_curve[i].t = (i - 2) * t_step_;

  // compute the last of last frame
  auto& p0 = st_curve[0];
  auto& p1 = st_curve[1];
  auto& p2 = st_curve[2];
  p2.s = 0.0;
  p1.s = p2.s - start_v * (p2.t - p1.t);
  auto v1 = start_v - start_a * (p2.t - p1.t);
  p0.s = p1.s - v1 * (p1.t - p0.t);

  // init s
  for (std::size_t i = 3; i < st_curve.size() - 3; ++i)
    st_curve[i].s = std::min(start_v * st_curve[i].t, s_max_);
  for (std::size_t i = st_curve.size() - 3; i < st_curve.size(); ++i)
    st_curve[i].s = std::numeric_limits<double>::max() / 2.0;

  // compute_cost func
  auto compute_cost = [&](const StPoint& p0, const StPoint& p1,
                          const StPoint& p2, const StPoint& p3,
                          const Constraint& con, bool get_s, bool get_v,
                          bool get_a, bool get_da) {
    auto& p = p3;
    double cost = 0.0;

    // get v
    auto v = (p3.s - p2.s) / (p3.t - p2.t);

    // get a
    auto v2 = (p2.s - p1.s) / (p2.t - p1.t);
    auto a = (v - v2) / (p3.t - p2.t);

    // get jerk
    auto v1 = (p1.s - p0.s) / (p1.t - p0.t);
    auto a2 = (v2 - v1) / (p2.t - p1.t);
    auto da = std::abs((a - a2) / (p3.t - p2.t));

    // cost for s
    if (get_s) {
      auto s = p.s;
      if (s > con.s_max)
        cost += kCostThreshold + kCostSMaxRatio * (s - con.s_max);
      /*else if (!std::isinf(con.s_preffered))
        cost += kCostSPrefferedRatio *
                std::abs((s - con.s_preffered) / (s + con.s_preffered));*/
      else if (s > con.s_preffered)
        cost += kCostSPrefferedRatio * (s - con.s_preffered);
      //
      if (TEST_P) std::cout << "s: " << s << " " << cost << std::endl;
    }

    // cost for v
    if (get_v) {
      if (v > con.v_max)
        cost += kCostThreshold + kCostVMaxRatio * (v - con.v_max);
      else if (v < 0.0)
        cost += kCostThreshold;
      else if (!std::isinf(con.v_preffered))
        cost += kCostVPrefferedRatio * std::abs(v - con.v_preffered);
      //
      if (TEST_P)
        std::cout << "v: " << v << " " << con.v_preffered << " " << cost
                  << std::endl;
    }

    // cost for a
    if (get_a) {
      if (a >= 0.0) {
        auto a_max = con.a_max;
        auto a_preffered = con.a_preffered;
        if (a > a_max)
          cost += kCostThreshold + kCostAMaxRatio * (a - a_max);
        else if (a > a_preffered)
          cost += kCostAPrefferedRatio * (a - a_preffered);
      } else {
        auto b_max = con.b_max;
        auto b_preffered = con.b_preffered;
        if (-a > b_max)
          cost += kCostThreshold + kCostAMaxRatio * (-a - b_max);
        else if (-a > b_preffered)
          cost += kCostAPrefferedRatio * (-a - b_preffered);
      }
      //
      if (TEST_P) std::cout << "a: " << a << " " << cost << std::endl;
    }

    // cost for jerk
    if (get_da) {
      auto da_max = con.da_max;
      auto da_preffered = con.da_preffered;
      if (da > da_max)
        cost += kCostThreshold + kCostJerkMaxRatio * (da - da_max);
      else if (da > da_preffered)
        cost += kCostJerkPrefferedRatio * (da - da_preffered);
      //
      if (TEST_P)
        std::cout << "da: " << da << " " << cost << std::endl << std::endl;
    }

    return cost;
  };

  // optimize_curve func
  auto optimize_curve = [&](const StPoint* input, std::size_t len,
                            StPoint* output) {
    // forward optimize func
    auto optimize_point = [&](std::size_t i, bool bidirection) {
      // get s funcs
      auto get_s_from_cur_v = [](double t0, double t1, double s0, double v1) {
        return v1 * (t1 - t0) + s0;
      };

      auto get_s_from_next_v = [](double t0, double t1, double s1, double v1) {
        return s1 - v1 * (t1 - t0);
      };

      auto get_s_from_cur_a = [](double t0, double t1, double t2, double s0,
                                 double s1, double a2) {
        auto t01 = t0 - t1;
        auto t12 = t1 - t2;
        auto t02 = t0 - t2;
        return (a2 * t01 * t12 * t12 - s0 * t12 + s1 * t02) / t01;
      };

      auto get_s_from_next_a = [](double t0, double t1, double t2, double s0,
                                  double s2, double a2) {
        auto t01 = t0 - t1;
        auto t12 = t1 - t2;
        auto t02 = t0 - t2;
        return (-a2 * t01 * t12 * t12 + s0 * t12 + s2 * t01) / t02;
      };

      auto get_s_from_nnext_a = [](double t0, double t1, double t2, double s1,
                                   double s2, double a2) {
        auto t01 = t0 - t1;
        auto t12 = t1 - t2;
        auto t02 = t0 - t2;
        return (a2 * t01 * t12 * t12 + s1 * t02 - s2 * t01) / t12;
      };

      auto get_s_from_cur_da = [](double t0, double t1, double t2, double t3,
                                  double s0, double s1, double s2, double da3) {
        auto t01 = t0 - t1;
        auto t12 = t1 - t2;
        auto t122 = t12 * t12;
        auto t23 = t2 - t3;
        auto t232 = t23 * t23;
        auto t233 = t232 * t23;
        auto t13 = t1 - t3;
        auto r0 = t01 * t122;
        return (-da3 * r0 * t233 + s2 * r0 + (s0 - s1) * t12 * t232 -
                (s1 - s2) * t01 * t13 * t23) /
               r0;
      };

      auto get_s_from_next_da = [](double t0, double t1, double t2, double t3,
                                   double s0, double s1, double s3,
                                   double da3) {
        auto t01 = t0 - t1;
        auto t12 = t1 - t2;
        auto t122 = t12 * t12;
        auto t23 = t2 - t3;
        auto t232 = t23 * t23;
        auto t233 = t232 * t23;
        auto t13 = t1 - t3;
        auto r0 = t01 * t122;
        auto r1 = t01 * t13 * t23;
        return (da3 * r0 * t233 + s1 * r1 + s3 * r0 - (s0 - s1) * t12 * t232) /
               (r0 + r1);
      };

      auto get_s_from_nnext_da = [](double t0, double t1, double t2, double t3,
                                    double s0, double s2, double s3,
                                    double da3) {
        auto t01 = t0 - t1;
        auto t12 = t1 - t2;
        auto t122 = t12 * t12;
        auto t23 = t2 - t3;
        auto t232 = t23 * t23;
        auto t233 = t232 * t23;
        auto t13 = t1 - t3;
        auto r0 = t01 * t122;
        auto r1 = t12 * t232;
        auto r2 = t01 * t13 * t23;
        return (-da3 * r0 * t233 + s0 * r1 + s2 * r2 + (s2 - s3) * r0) /
               (r1 + r2);
      };

      auto get_s_from_nnnext_da = [](double t0, double t1, double t2, double t3,
                                     double s1, double s2, double s3,
                                     double da3) {
        auto t01 = t0 - t1;
        auto t12 = t1 - t2;
        auto t122 = t12 * t12;
        auto t23 = t2 - t3;
        auto t232 = t23 * t23;
        auto t233 = t232 * t23;
        auto t13 = t1 - t3;
        auto r0 = t01 * t122;
        auto r1 = t12 * t232;
        return (da3 * r0 * t233 + s1 * r1 + (s1 - s2) * t01 * t13 * t23 -
                (s2 - s3) * r0) /
               r1;
      };

      // get constraint func
      auto get_con = [&](std::size_t i, const StPoint& p) {
        auto con = t_constraint_[p.t / t_step_];
        const auto& pprev = input[i - 2];
        const auto& prev = input[i - 1];
        auto v = (p.s - prev.s) / (p.t - prev.t);
        auto v0 = (prev.s - pprev.s) / (prev.t - pprev.t);
        auto a = (v - v0) / (p.t - prev.t);
        auto cap = get_vehicle_capability(p.t, p.s, v, a);

        CombineConstraint(s_constraint_[p.s / s_step_], &con);
        con.a_max = std::min(cap.a_max, con.a_max);
        con.a_preffered = std::min(con.a_max, con.a_preffered);
        con.b_max = std::min(cap.b_max, con.b_max);
        con.b_preffered = std::min(con.b_max, con.b_preffered);
        return con;
      };

      const auto& ppprev = input[i - 3];
      const auto& pprev = input[i - 2];
      const auto& prev = input[i - 1];
      auto& cur = input[i];
      const auto& next = input[i + 1];
      const auto& nnext = input[i + 2];
      const auto& nnnext = input[i + 3];

      // s_max and s_min
      auto s_max = std::min(next.s, s_max_);
      auto s_min = prev.s;
      constexpr double kSAdjustRatio = 1e-5;
      auto ds = (s_max - s_min) * kSAdjustRatio;
      s_max -= ds;
      s_min += ds;

      //
      double step = 0.0;

      // find the lowest cost
      struct {
        double s = std::numeric_limits<double>::max();
        double cost = std::numeric_limits<double>::max();
      } choices[2];

      auto update_choices = [&](double s) {
        constexpr double kSEpsilon = 1e-3;
        if (std::abs(s - choices[0].s) < kSEpsilon ||
            std::abs(s - choices[1].s) < kSEpsilon)
          return;

        StPoint p;
        p.t = cur.t;
        p.s = s;

        auto cur_cost = compute_cost(ppprev, pprev, prev, p, get_con(i, p),
                                     true, true, true, true);
        auto next_cost = 0.0;
        auto nnext_cost = 0.0;
        auto nnnext_cost = 0.0;
        if (bidirection) {
          if (i + 1 < len)
            next_cost = compute_cost(pprev, prev, p, next, get_con(i + 1, next),
                                     false, true, true, true);
          if (i + 2 < len)
            nnext_cost =
                compute_cost(prev, p, next, nnext, get_con(i + 2, nnext), false,
                             false, true, true);
          if (i + 3 < len)
            nnnext_cost =
                compute_cost(p, next, nnext, nnnext, get_con(i + 3, nnnext),
                             false, false, false, true);
        }

        auto cost =
            cur_cost + next_cost * 0.5 + nnext_cost * 0.0 + nnnext_cost * 0.0;
        if (cost <= choices[0].cost) {
          choices[1] = choices[0];
          choices[0].s = s;
          choices[0].cost = cost;
        } else if (cost <= choices[1].cost) {
          choices[1].s = s;
          choices[1].cost = cost;
        }
      };

      auto filter_update_choices = [&](double s) {
        if (s > s_min && s < s_max) update_choices(s);
      };

      update_choices(s_max);
      update_choices(s_min);
      // filter_update_choices(cur.s);
      //
      if (TEST_P1)
        std::cout << "----------------- s0:" << choices[0].s
                  << " s1:" << choices[1].s << std::endl;
      auto cur_con = get_con(i, cur);
      filter_update_choices(cur_con.s_max);
      filter_update_choices(cur_con.s_preffered);
      filter_update_choices(
          get_s_from_cur_v(prev.t, cur.t, prev.s, cur_con.v_max));
      filter_update_choices(
          get_s_from_cur_v(prev.t, cur.t, prev.s, cur_con.v_preffered));
      filter_update_choices(get_s_from_cur_a(pprev.t, prev.t, cur.t, pprev.s,
                                             prev.s, cur_con.a_max));
      filter_update_choices(get_s_from_cur_a(pprev.t, prev.t, cur.t, pprev.s,
                                             prev.s, cur_con.a_preffered));
      filter_update_choices(get_s_from_cur_a(pprev.t, prev.t, cur.t, pprev.s,
                                             prev.s, -cur_con.b_max));
      filter_update_choices(get_s_from_cur_a(pprev.t, prev.t, cur.t, pprev.s,
                                             prev.s, -cur_con.b_preffered));
      filter_update_choices(get_s_from_cur_da(ppprev.t, pprev.t, prev.t, cur.t,
                                              ppprev.s, pprev.s, prev.s,
                                              cur_con.da_max));
      filter_update_choices(get_s_from_cur_da(ppprev.t, pprev.t, prev.t, cur.t,
                                              ppprev.s, pprev.s, prev.s,
                                              cur_con.da_preffered));
      filter_update_choices(get_s_from_cur_da(ppprev.t, pprev.t, prev.t, cur.t,
                                              ppprev.s, pprev.s, prev.s,
                                              -cur_con.da_max));
      filter_update_choices(get_s_from_cur_da(ppprev.t, pprev.t, prev.t, cur.t,
                                              ppprev.s, pprev.s, prev.s,
                                              -cur_con.da_preffered));
      if (bidirection) {
        if (i + 1 < len) {
          auto next_con = get_con(i + 1, next);
          filter_update_choices(
              get_s_from_next_v(cur.t, next.t, next.s, next_con.v_max));
          filter_update_choices(
              get_s_from_next_v(cur.t, next.t, next.s, next_con.v_preffered));
          filter_update_choices(get_s_from_next_a(prev.t, cur.t, next.t, prev.s,
                                                  next.s, next_con.a_max));
          filter_update_choices(get_s_from_next_a(
              prev.t, cur.t, next.t, prev.s, next.s, next_con.a_preffered));
          filter_update_choices(get_s_from_next_a(prev.t, cur.t, next.t, prev.s,
                                                  next.s, -next_con.b_max));
          filter_update_choices(get_s_from_next_a(
              prev.t, cur.t, next.t, prev.s, next.s, -next_con.b_preffered));
          filter_update_choices(get_s_from_next_da(pprev.t, prev.t, cur.t,
                                                   next.t, pprev.s, prev.s,
                                                   next.s, next_con.da_max));
          filter_update_choices(
              get_s_from_next_da(pprev.t, prev.t, cur.t, next.t, pprev.s,
                                 prev.s, next.s, next_con.da_preffered));
          filter_update_choices(get_s_from_next_da(pprev.t, prev.t, cur.t,
                                                   next.t, pprev.s, prev.s,
                                                   next.s, -next_con.da_max));
          filter_update_choices(
              get_s_from_next_da(pprev.t, prev.t, cur.t, next.t, pprev.s,
                                 prev.s, next.s, -next_con.da_preffered));
        }
        if (i + 2 < len) {
          auto nnext_con = get_con(i + 2, nnext);
          filter_update_choices(get_s_from_nnext_a(
              cur.t, next.t, nnext.t, next.s, nnext.s, nnext_con.a_max));
          filter_update_choices(get_s_from_nnext_a(
              cur.t, next.t, nnext.t, next.s, nnext.s, nnext_con.a_preffered));
          filter_update_choices(get_s_from_nnext_a(
              cur.t, next.t, nnext.t, next.s, nnext.s, -nnext_con.b_max));
          filter_update_choices(get_s_from_nnext_a(
              cur.t, next.t, nnext.t, next.s, nnext.s, -nnext_con.b_preffered));
          filter_update_choices(get_s_from_nnext_da(prev.t, cur.t, next.t,
                                                    nnext.t, prev.s, next.s,
                                                    nnext.s, nnext_con.da_max));
          filter_update_choices(
              get_s_from_nnext_da(prev.t, cur.t, next.t, nnext.t, prev.s,
                                  next.s, nnext.s, nnext_con.da_preffered));
          filter_update_choices(
              get_s_from_nnext_da(prev.t, cur.t, next.t, nnext.t, prev.s,
                                  next.s, nnext.s, -nnext_con.da_max));
          filter_update_choices(
              get_s_from_nnext_da(prev.t, cur.t, next.t, nnext.t, prev.s,
                                  next.s, nnext.s, -nnext_con.da_preffered));
        }
        if (i + 3 < len) {
          auto nnnext_con = get_con(i + 3, nnnext);
          filter_update_choices(
              get_s_from_nnnext_da(cur.t, next.t, nnext.t, nnnext.t, next.s,
                                   nnext.s, nnnext.s, nnnext_con.da_max));
          filter_update_choices(
              get_s_from_nnnext_da(cur.t, next.t, nnext.t, nnnext.t, next.s,
                                   nnext.s, nnnext.s, nnnext_con.da_preffered));
          filter_update_choices(
              get_s_from_nnnext_da(cur.t, next.t, nnext.t, nnnext.t, next.s,
                                   nnext.s, nnnext.s, -nnnext_con.da_max));
          filter_update_choices(get_s_from_nnnext_da(
              cur.t, next.t, nnext.t, nnnext.t, next.s, nnext.s, nnnext.s,
              -nnnext_con.da_preffered));
        }
      }

      for (auto j = 0; j < kFindCycleTimes; ++j) {
        auto s_upper = std::max(choices[0].s, choices[1].s);
        auto s_lower = std::min(choices[0].s, choices[1].s);
        //
        if (TEST_P1)
          std::cout << "s_upper:" << s_upper << " s_lower:" << s_lower
                    << std::endl;

        constexpr int kStepNum = 2;
        auto step = (s_upper - s_lower) / kStepNum;
        for (auto k = 1; k < kStepNum; ++k) update_choices(s_lower + k * step);
      }

      // assign new s
      output[i].s = choices[0].s;
      //
      if (TEST_P1)
        std::cout << "----------------- " << s_min << " " << s_max << " "
                  << output[i].s << std::endl;
    };

    for (std::size_t i = 0; i < len; i += 4) optimize_point(i, true);
    for (std::size_t i = 1; i < len; i += 4) optimize_point(i, true);
    for (std::size_t i = 2; i < len; i += 4) optimize_point(i, true);
    for (std::size_t i = 3; i < len; i += 4) optimize_point(i, true);
  };

  // resample func
  auto resample_st_curve = [&](std::size_t start_i, std::size_t len) {
    std::vector<StPoint> new_st_curve;
    new_st_curve.resize(st_curve.size() + len);
    for (std::size_t i = 0; i < 2 + start_i; ++i) new_st_curve[i] = st_curve[i];
    for (std::size_t i = 0; i < len * 2; i += 2) {
      const auto& cur = st_curve[2 + start_i + i / 2];
      const auto& next = st_curve[2 + start_i + i / 2 + 1];
      auto& p0 = new_st_curve[2 + start_i + i];
      auto& p1 = new_st_curve[2 + start_i + i + 1];
      p0.t = cur.t;
      p0.s = cur.s;
      p1.t = (cur.t + next.t) / 2.0;
      p1.s = (cur.s + next.s) / 2.0;
    }
    for (auto i = 2 + start_i + len * 2; i - len < st_curve.size(); ++i)
      new_st_curve[i] = st_curve[i - len];
    st_curve = std::move(new_st_curve);
    process_buf = &st_curve[3 + start_i];
    process_len += len;
  };

  // iterative optimize
  for (auto i = (decltype(iter_num))0; i < iter_num; ++i)
    optimize_curve(process_buf, process_len, process_buf);
  for (decltype(resampling) i = 0; i < resampling; ++i) {
    resample_st_curve(0, process_len / 2);
    for (auto i = (decltype(iter_num))0; i < iter_num; ++i)
      optimize_curve(process_buf, process_len, process_buf);
  }

  // is it satisfied constraints
  /*bool unsatisfied = false;
  for (size_t i = 0; i < buf_len; ++i) {
    if (compute_cost(output_buf, buf_len, output_buf[i], i) > kCostThreshold) {
      unsatisfied = true;
      break;
    }
  }
  if (unsatisfied) {
    AERROR << "failure to satisfy the constraints.";
    return Status(ErrorCode::PLANNING_ERROR,
                  "failure to satisfy the constraints.");
  }*/

  // output
  output->resize(process_len + 1);
  auto& first_o = (*output)[0];
  first_o.t = 0.0;
  first_o.s = 0.0;
  first_o.v = start_v;
  first_o.a = start_a;
  first_o.da = start_da;
  for (size_t i = 1; i < output->size(); ++i) {
    const auto& p = process_buf[i - 1];
    auto& cur = (*output)[i];
    cur.t = p.t;
    cur.s = p.s;
  }
  for (size_t i = 1; i < output->size(); ++i) {
    const auto& prev = (*output)[i - 1];
    auto& cur = (*output)[i];

    // ignore the tiny value
    constexpr double kZeroVelocityThreshold = 1e-3;
    auto v = (cur.s - prev.s) / (cur.t - prev.t);
    if (v <= kZeroVelocityThreshold) {
      cur.s = prev.s;
    } else {
      constexpr double kZeroAccelerationThreshold = 1e-2;
      auto a =
          ((cur.s - prev.s) / (cur.t - prev.t) - prev.v) / (cur.t - prev.t);
      if (std::abs(a) <= kZeroAccelerationThreshold)
        cur.s = prev.v * (cur.t - prev.t) + prev.s;
    }

    cur.v = (cur.s - prev.s) / (cur.t - prev.t);
    cur.a = (cur.v - prev.v) / (cur.t - prev.t);
    cur.da = (cur.a - prev.a) / (cur.t - prev.t);
  }

  return Status::OK();
}

}  // namespace navi_st_solver
}  // namespace planning
}  // namespace apollo
