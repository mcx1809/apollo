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
 * @file navi_st_solver_test.cc
 **/

#include "modules/planning/navi/decider/navi_st_solver.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace apollo {
namespace planning {
namespace navi_st_solver {

using apollo::common::Status;

TEST(Solver, StartFromZero) {
  Solver solver;
  solver.Reset(1.0, 1000.0, 1.0, 50.0);

  Constraint con;
  // con.s_preffered = 5.0;
  con.v_max = 30.0;
  con.v_preffered = 20.0;
  con.a_preffered = 1.0;
  con.b_preffered = 1.0;
  con.da_preffered = 3.0;
  con.da_max = 6.0;
  solver.UpdateConstraint(con);

  con.v_max = 10.0;
  con.v_preffered = 5.0;
  // solver.UpdateConstraintForStation(100.0, 200.0, con);

  con.v_max = 20.0;
  con.v_preffered = 10.0;
  solver.UpdateConstraintForTime(20.0, 30.0, con);

  auto get_vehicle_capability = [](double t, double s, double v, double a) {
    auto cap = VehicleCapability();
    if (v < 10.0)
      cap.a_max = 5.0;
    else
      cap.a_max = 5.0;
    cap.b_max = 5.0;
    return cap;
  };
  std::vector<OutputPoint> points;
  EXPECT_EQ(Status::OK(), solver.Solve(0.0, 0.0, 0.0, get_vehicle_capability,
                                       1000, 0, &points));
  // EXPECT_NEAR(0.0, points.front().s, 0.1);
  // EXPECT_NEAR(0.0, points.front().t, 0.1);
  // EXPECT_NEAR(0.0, points.front().v, 0.1);
  // EXPECT_NEAR(25.0, points[25].s, 0.1);
  // EXPECT_NEAR(5.0, points[25].t, 0.1);
  // EXPECT_NEAR(10.0, points[25].v, 0.1);
  for (const auto& p : points) {
    // if (point.s > 25.0) EXPECT_NEAR(10.0, point.v, 0.1);
    std::cout.setf(std::ios::left);
    std::cout << "t:";
    std::cout.width(15);
    std::cout << p.t;
    std::cout << "s:";
    std::cout.width(15);
    std::cout << p.s;
    std::cout << "v:";
    std::cout.width(15);
    std::cout << p.v;
    std::cout << "a:";
    std::cout.width(15);
    std::cout << p.a;
    std::cout << "da:";
    std::cout.width(15);
    std::cout << p.da << std::endl;
  }
  EXPECT_TRUE(false);
}

/*TEST(NaviSpeedTsGraph, Solve2) {
  NaviSpeedTsGraph graph;
  graph.Reset(1.0, 100.0, 0.0, 0.0, 0.0);
  auto get_safe_distance = [](double v) { return 1.0 * v + 2.0; };

  NaviSpeedTsConstraints constraints;
  constraints.v_max = 20.0;
  constraints.v_preffered = 10.0;
  constraints.a_max = 4.0;
  constraints.a_preffered = 2.0;
  constraints.b_max = 5.0;
  constraints.b_preffered = 2.0;
  graph.UpdateConstraints(constraints);

  graph.UpdateObstacleConstraints(40.0, get_safe_distance(0.0), 0.5, 0.0, 10.0);

  std::vector<NaviSpeedTsPoint> points;
  EXPECT_EQ(Status::OK(), graph.Solve(&points));
  EXPECT_NEAR(0.0, points.front().s, 0.1);
  EXPECT_NEAR(0.0, points.front().t, 0.1);
  EXPECT_NEAR(0.0, points.front().v, 0.1);
  for (const auto& point : points)
    if (point.s > 38.0) EXPECT_NEAR(0.0, point.v, 0.1);
  EXPECT_NEAR(0.0, points.back().v, 0.1);
}

TEST(NaviSpeedTsGraph, Solve3) {
  NaviSpeedTsGraph graph;
  graph.Reset(1.0, 100.0, 5.0, 0.0, 0.0);
  auto get_safe_distance = [](double v) { return 1.0 * v + 2.0; };

  NaviSpeedTsConstraints constraints;
  constraints.v_max = 20.0;
  constraints.v_preffered = 10.0;
  constraints.a_max = 4.0;
  constraints.a_preffered = 2.0;
  constraints.b_max = 5.0;
  constraints.b_preffered = 2.0;
  graph.UpdateConstraints(constraints);

  graph.UpdateObstacleConstraints(10.0, get_safe_distance(5.0), 0.5, 5.0, 10.0);

  std::vector<NaviSpeedTsPoint> points;
  EXPECT_EQ(Status::OK(), graph.Solve(&points));
  EXPECT_NEAR(0.0, points.front().s, 0.1);
  EXPECT_NEAR(0.0, points.front().t, 0.1);
  EXPECT_NEAR(5.0, points.front().v, 0.1);
  for (const auto& point : points) {
    if (point.s > 15.0) {
      auto obstacle_distance = 5.0 * point.t + 10.0;
      EXPECT_GE(obstacle_distance, point.s);
      EXPECT_NEAR(5.0, point.v, 0.1);
    }
  }
}

TEST(NaviSpeedTsGraph, ErrorTest) {}*/

}  // namespace navi_st_solver
}  // namespace planning
}  // namespace apollo
