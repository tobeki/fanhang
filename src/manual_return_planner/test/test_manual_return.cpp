#include "manual_return_planner/manual_return_planner.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {
using manual_return_planner::RdpSimplifier;
using manual_return_planner::TrajectoryPoint;

TrajectoryPoint point(double x, double y, double z = 0.0) {
  TrajectoryPoint result;
  result.position << x, y, z;
  return result;
}
}

TEST(ManualReturnRdp, CompressesNearlyCollinearLine) {
  std::vector<TrajectoryPoint> input;
  for (int i = 0; i < 100; ++i) input.push_back(point(0.1 * i, 0.001 * std::sin(i), 0.0));
  double deviation = 0.0;
  const std::vector<TrajectoryPoint> output = RdpSimplifier::simplify(input, 0.02, &deviation);
  EXPECT_EQ(output.front().position, input.front().position);
  EXPECT_EQ(output.back().position, input.back().position);
  EXPECT_LT(output.size(), 10u);
  EXPECT_LE(deviation, 0.02 + 1e-9);
}

TEST(ManualReturnRdp, PreservesNinetyDegreeCorner) {
  std::vector<TrajectoryPoint> input;
  for (int i = 0; i <= 10; ++i) input.push_back(point(i, 0.0));
  for (int i = 1; i <= 10; ++i) input.push_back(point(10.0, i));
  const std::vector<TrajectoryPoint> output = RdpSimplifier::simplify(input, 0.05, nullptr);
  ASSERT_GE(output.size(), 3u);
  bool found_corner = false;
  for (const TrajectoryPoint& p : output)
    found_corner = found_corner || (std::abs(p.position.x() - 10.0) < 1e-9 &&
                                    std::abs(p.position.y()) < 1e-9);
  EXPECT_TRUE(found_corner);
}

TEST(ManualReturnRdp, UsesThreeDimensionalDistance) {
  std::vector<TrajectoryPoint> input;
  input.push_back(point(0.0, 0.0, 0.0));
  input.push_back(point(0.5, 0.0, 0.4));
  input.push_back(point(1.0, 0.0, 0.0));
  const std::vector<TrajectoryPoint> output = RdpSimplifier::simplify(input, 0.1, nullptr);
  ASSERT_EQ(output.size(), 3u);
}

TEST(ManualReturnPreprocessor, RejectsBackwardTime) {
  std::vector<TrajectoryPoint> input{point(0.0, 0.0), point(1.0, 0.0)};
  input[0].timestamp = 2.0;
  input[1].timestamp = 1.0;
  manual_return_planner::TrajectoryPreprocessor preprocessor;
  manual_return_planner::ManualReturnConfig config;
  std::vector<TrajectoryPoint> output;
  std::string warning;
  manual_return_planner::ManualReturnStatus status;
  EXPECT_FALSE(preprocessor.preprocess(input, config, &output, &warning, &status));
  EXPECT_EQ(status, manual_return_planner::ManualReturnStatus::INVALID_TIMESTAMP);
}

TEST(ManualReturnRdp, PreservesSShapeGeometry) {
  std::vector<TrajectoryPoint> input;
  for (int i = 0; i <= 20; ++i) {
    const double x = 0.1 * i;
    input.push_back(point(x, 0.25 * std::sin(2.0 * x), 0.05 * x));
    input.back().timestamp = 0.1 * i;
  }
  double deviation = 0.0;
  const std::vector<TrajectoryPoint> output = RdpSimplifier::simplify(input, 0.03, &deviation);
  ASSERT_GE(output.size(), 3u);
  EXPECT_LE(deviation, 0.03 + 1e-9);
  EXPECT_EQ(output.front().position, input.front().position);
  EXPECT_EQ(output.back().position, input.back().position);
}

TEST(ManualReturnPlanner, ReversesEndpointsAndBoundsSegments) {
  std::vector<TrajectoryPoint> input;
  for (int i = 0; i <= 20; ++i) {
    TrajectoryPoint p = point(static_cast<double>(i), 0.0, 1.0);
    p.timestamp = 0.1 * i;
    input.push_back(p);
  }
  manual_return_planner::ManualReturnConfig config;
  config.rdp_epsilon = 0.05;
  config.max_segment_length = 5.0;
  manual_return_planner::ManualReturnPlanner planner;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr no_map;
  const auto result = planner.planManualReturn(input, no_map, config);
  ASSERT_TRUE(result.status == manual_return_planner::ManualReturnStatus::SUCCESS_RDP ||
              result.status == manual_return_planner::ManualReturnStatus::SUCCESS_DENSE_BACKTRACK);
  ASSERT_FALSE(result.return_waypoints.empty());
  EXPECT_NEAR(result.return_waypoints.front().position.x(), 20.0, 1e-9);
  EXPECT_NEAR(result.return_waypoints.back().position.x(), 0.0, 1e-9);
  for (std::size_t i = 1; i < result.return_waypoints.size(); ++i)
    EXPECT_LE((result.return_waypoints[i].position - result.return_waypoints[i - 1].position).norm(),
              config.max_segment_length + 1e-9);
}

TEST(ManualReturnPlanner, UsesLowLandingHandoffAcrossTakeoffClimb) {
  std::vector<TrajectoryPoint> input;
  input.push_back(point(0.0, 0.0, 0.0));
  input.back().yaw = 1.2;
  input.push_back(point(0.0, 0.0, 0.5));
  input.push_back(point(0.0, 0.0, 1.0));
  input.push_back(point(2.0, 0.0, 1.0));
  input.back().yaw = 0.7;
  for (std::size_t i = 0; i < input.size(); ++i)
    input[i].timestamp = 0.1 * static_cast<double>(i);

  manual_return_planner::ManualReturnConfig config;
  manual_return_planner::ManualReturnPlanner planner;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr no_map;
  const auto result = planner.planManualReturn(input, no_map, config);

  ASSERT_TRUE(result.status == manual_return_planner::ManualReturnStatus::SUCCESS_RDP ||
              result.status == manual_return_planner::ManualReturnStatus::SUCCESS_DENSE_BACKTRACK);
  ASSERT_FALSE(result.return_waypoints.empty());
  EXPECT_NEAR(result.return_waypoints.back().position.x(),
              input.front().position.x(), 1e-9);
  EXPECT_NEAR(result.return_waypoints.back().position.y(),
              input.front().position.y(), 1e-9);
  EXPECT_NEAR(result.return_waypoints.back().position.z(),
              input.front().position.z() + config.landing_handoff_height,
              1e-9);
  EXPECT_TRUE(result.landing_yaw_transition_applied);
  EXPECT_NEAR(result.return_waypoints.back().yaw, input.front().yaw, 1e-9);
  for (std::size_t i = 0; i + 1 < result.return_waypoints.size(); ++i)
    EXPECT_NEAR(result.return_waypoints[i].yaw, input.back().yaw, 1e-9);
}

TEST(ManualReturnPlanner, RestoresMeasuredSamplesAcrossHeightChange) {
  std::vector<TrajectoryPoint> input;
  // A mostly vertical 1 m climb with the small horizontal drift that a pilot
  // or controller can produce in real flight.
  for (int i = 0; i <= 10; ++i) {
    TrajectoryPoint p = point(0.01 * i, 0.0, 0.1 * i);
    p.timestamp = 0.1 * i;
    input.push_back(p);
  }
  input.push_back(point(1.0, 0.0, 1.0));
  input.back().timestamp = 1.1;
  input.push_back(point(2.0, 0.0, 1.0));
  input.back().timestamp = 1.2;

  manual_return_planner::ManualReturnConfig config;
  config.landing_handoff_height = 0.0;
  config.vertical_preserve_threshold = 0.05;
  config.safe_rdp.enabled = false;
  // This test exercises the Z guard independently of Home startup trimming.
  config.home_trim_radius = 100.0;
  manual_return_planner::ManualReturnPlanner planner;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr no_map;
  const auto result = planner.planManualReturn(input, no_map, config);

  ASSERT_TRUE(result.status == manual_return_planner::ManualReturnStatus::SUCCESS_RDP ||
              result.status == manual_return_planner::ManualReturnStatus::SUCCESS_DENSE_BACKTRACK);
  EXPECT_GT(result.vertical_protected_segments, 0u);
  EXPECT_GT(result.vertical_restored_points, 0u);
  // The protected interval retains structural boundaries, not every 10 Hz
  // sample. The final Home boundary must remain present.
  EXPECT_LT(result.return_waypoints.size(), input.size());
  EXPECT_NEAR(result.return_waypoints.back().position.x(), 0.0, 1e-9);
  EXPECT_NEAR(result.return_waypoints.back().position.z(), 0.0, 1e-9);
  EXPECT_LT(result.vertical_restored_points, 10u);
}

TEST(ManualReturnPlanner, StillCompressesLevelFlightWithSmallZNoise) {
  std::vector<TrajectoryPoint> input;
  for (int i = 0; i <= 100; ++i) {
    TrajectoryPoint p = point(0.1 * i, 0.0,
                              0.01 * std::sin(0.2 * i));
    p.timestamp = 0.1 * i;
    input.push_back(p);
  }
  manual_return_planner::ManualReturnConfig config;
  config.landing_handoff_height = 0.0;
  config.vertical_preserve_threshold = 0.05;
  config.safe_rdp.enabled = false;
  manual_return_planner::ManualReturnPlanner planner;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr no_map;
  const auto result = planner.planManualReturn(input, no_map, config);

  ASSERT_TRUE(result.status == manual_return_planner::ManualReturnStatus::SUCCESS_RDP ||
              result.status == manual_return_planner::ManualReturnStatus::SUCCESS_DENSE_BACKTRACK);
  EXPECT_EQ(result.vertical_protected_segments, 0u);
  EXPECT_EQ(result.vertical_restored_points, 0u);
  EXPECT_LE(result.return_waypoints.size(), 5u);
  EXPECT_LT(result.return_waypoints.size(), input.size() / 10);
}

TEST(ManualReturnPlanner, WholeMapRejectsShortcutAndKeepsReturnAvailable) {
  std::vector<TrajectoryPoint> input = {
      point(0.0, 0.0, 0.0),
      point(1.0, 1.0, 0.0),
      point(2.0, 0.0, 0.0),
  };
  for (std::size_t i = 0; i < input.size(); ++i)
    input[i].timestamp = static_cast<double>(i);

  pcl::PointCloud<pcl::PointXYZ>::Ptr map(
      new pcl::PointCloud<pcl::PointXYZ>);
  map->push_back(pcl::PointXYZ(1.0f, 0.0f, 0.0f));
  manual_return_planner::ManualReturnConfig config;
  config.rdp_epsilon = 2.0;  // force the direct chord as the RDP candidate
  config.landing_handoff_height = 0.0;
  config.safe_rdp.enabled = true;
  config.safe_rdp.trust_flown_history = false;
  manual_return_planner::ManualReturnPlanner planner;
  const auto result = planner.planManualReturn(input, map, config);

  ASSERT_TRUE(result.status == manual_return_planner::ManualReturnStatus::SUCCESS_RDP ||
              result.status == manual_return_planner::ManualReturnStatus::SUCCESS_DENSE_BACKTRACK);
  EXPECT_EQ(result.shortcut_candidates, 1);
  EXPECT_EQ(result.unsafe_segments, 1);
  EXPECT_EQ(result.map_restored_points, 1u);
  ASSERT_EQ(result.return_waypoints.size(), input.size());
  EXPECT_EQ(result.return_waypoints[1].position, input[1].position);
}

TEST(ManualReturnPreprocessor, RetainsNonFiniteAuxiliaryAsWarning) {
  std::vector<TrajectoryPoint> input{point(0.0, 0.0), point(1.0, 0.0)};
  input[0].timestamp = 0.0;
  input[1].timestamp = 0.1;
  input[1].velocity.x() = std::numeric_limits<double>::quiet_NaN();
  manual_return_planner::TrajectoryPreprocessor preprocessor;
  manual_return_planner::ManualReturnConfig config;
  std::vector<TrajectoryPoint> output;
  std::string warning;
  manual_return_planner::ManualReturnStatus status;
  EXPECT_TRUE(preprocessor.preprocess(input, config, &output, &warning, &status));
  EXPECT_FALSE(warning.empty());
  EXPECT_EQ(output.size(), 2u);
}
