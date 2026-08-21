#include "manual_return_planner/command_gate.h"
#include "manual_return_planner/manual_return_planner.h"

#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace {
using manual_return_planner::CommandGateCore;
using manual_return_planner::CommandSource;
using manual_return_planner::ManualReturnConfig;
using manual_return_planner::ManualReturnPlanner;
using manual_return_planner::TrajectoryPoint;

TrajectoryPoint point(double x, double y, double z = 0.0) {
  TrajectoryPoint result;
  result.position << x, y, z;
  return result;
}
}  // namespace

// Test A: NORMAL mode forwards normal and drops return commands.
TEST(CommandGate, NormalModeForwardsNormalDropsReturn) {
  CommandGateCore gate;
  EXPECT_EQ(gate.source(), CommandSource::NORMAL);
  EXPECT_TRUE(gate.shouldForward(CommandSource::NORMAL));
  EXPECT_FALSE(gate.shouldForward(CommandSource::MANUAL_RETURN));
}

// Test B: after switching to MANUAL_RETURN, return is forwarded and normal is
// dropped.
TEST(CommandGate, ReturnModeForwardsReturnDropsNormal) {
  CommandGateCore gate;
  gate.switchToManualReturn();
  EXPECT_EQ(gate.source(), CommandSource::MANUAL_RETURN);
  EXPECT_TRUE(gate.shouldForward(CommandSource::MANUAL_RETURN));
  EXPECT_FALSE(gate.shouldForward(CommandSource::NORMAL));
}

// Test C: arriving normal commands must never switch the source back.
TEST(CommandGate, NormalCommandDoesNotSwitchBack) {
  CommandGateCore gate;
  gate.switchToManualReturn();
  // A normal command arrives: it is dropped, but the gate must stay in return.
  EXPECT_FALSE(gate.shouldForward(CommandSource::NORMAL));
  EXPECT_EQ(gate.source(), CommandSource::MANUAL_RETURN);
  // Only an explicit reset returns to NORMAL.
  gate.switchToNormal();
  EXPECT_EQ(gate.source(), CommandSource::NORMAL);
  EXPECT_TRUE(gate.shouldForward(CommandSource::NORMAL));
}

// Test D: a trigger anchor appended at (or near) the latest odometry becomes
// the return start, even when it is within min_point_spacing of the last
// recorded sample (the preprocessor must not let it disappear).
TEST(CommandGate, TriggerAnchorBecomesReturnStart) {
  std::vector<TrajectoryPoint> input;
  for (int i = 0; i <= 10; ++i) {
    TrajectoryPoint p = point(0.1 * i, 0.0, 1.0);
    p.timestamp = 0.1 * i;
    input.push_back(p);
  }
  // The latest odometry at trigger is only 2 cm past the last 10 Hz sample,
  // i.e. below the default min_point_spacing of 3 cm.
  TrajectoryPoint anchor = point(1.02, 0.0, 1.0);
  anchor.timestamp = 1.03;
  input.push_back(anchor);

  ManualReturnConfig config;
  config.min_point_spacing = 0.03;
  config.rdp_epsilon = 0.05;
  config.max_segment_length = 5.0;

  ManualReturnPlanner planner;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr no_map;
  const auto result = planner.planManualReturn(input, no_map, config);

  ASSERT_FALSE(result.return_waypoints.empty());
  EXPECT_NEAR(result.return_waypoints.front().position.x(), anchor.position.x(),
              1e-9);
  EXPECT_NEAR(result.return_waypoints.front().position.y(), anchor.position.y(),
              1e-9);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
