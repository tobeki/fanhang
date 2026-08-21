#include "manual_return_planner/manual_return_planner.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace {
using manual_return_planner::ReturnMetricsAnalyzer;
using manual_return_planner::TrajectoryPoint;

TrajectoryPoint pt(double x, double y, double z = 0.0) {
  TrajectoryPoint p;
  p.position << x, y, z;
  return p;
}
}  // namespace

TEST(ReturnMetricsAnalyzer, PointToPolylineDistance) {
  std::vector<TrajectoryPoint> polyline{pt(0, 0), pt(1, 0)};
  const Eigen::Vector3d p(0, 1, 0);
  EXPECT_NEAR(
      ReturnMetricsAnalyzer::pointToPolylineDistance(p, polyline), 1.0, 1e-9);
}

// A simplified path that cuts a 90-degree corner should show a clear, bounded
// deviation from the actually-flown L-shaped corridor.
TEST(ReturnMetricsAnalyzer, DeviationDetectsCornerCut) {
  std::vector<TrajectoryPoint> history{pt(0, 0), pt(5, 0), pt(5, 5)};
  std::vector<TrajectoryPoint> simplified{pt(0, 0), pt(5, 5)};
  const auto d = ReturnMetricsAnalyzer::computePathDeviation(simplified, history);
  EXPECT_GT(d.max, 1.0);
  EXPECT_NEAR(d.max, 2.5, 0.2);
  EXPECT_GT(d.mean, 0.5);
  EXPECT_GT(d.p95, 1.0);
}

// An identical simplified path stays on the corridor, so deviation is zero.
TEST(ReturnMetricsAnalyzer, DeviationZeroForIdenticalPath) {
  std::vector<TrajectoryPoint> history;
  for (int i = 0; i <= 10; ++i) history.push_back(pt(0.1 * i, 0.0));
  const auto d = ReturnMetricsAnalyzer::computePathDeviation(history, history);
  EXPECT_NEAR(d.max, 0.0, 1e-9);
  EXPECT_NEAR(d.mean, 0.0, 1e-9);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
