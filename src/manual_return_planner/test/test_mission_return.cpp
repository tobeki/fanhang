#include "manual_return_planner/mission_return_planner.h"

#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using manual_return_planner::MissionReturnConfig;
using manual_return_planner::MissionReturnPlanner;
using manual_return_planner::MissionReturnStatus;
using manual_return_planner::MissionWaypoint;

namespace {
MissionWaypoint wp(double x, double y, double z, double yaw = 0.0,
                   bool key = false) {
  MissionWaypoint p;
  p.position << x, y, z;
  p.yaw = yaw;
  p.is_key_wp = key;
  return p;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr mapPoint(double x, double y, double z) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
  map->push_back(pcl::PointXYZ(x, y, z));
  return map;
}
}  // namespace

TEST(MissionReturnPlanner, UsesWholeMapAndFixedTakeoffYaw) {
  const std::vector<MissionWaypoint> mission = {
      wp(0.0, 0.0, -1.0, 1.2), wp(1.0, 0.0, -1.0), wp(2.0, 0.0, -1.0)};
  MissionReturnConfig config;
  MissionReturnPlanner planner;
  const auto result = planner.plan(mission, mapPoint(10.0, 10.0, 10.0), config);
  ASSERT_EQ(result.status, MissionReturnStatus::SUCCESS);
  ASSERT_EQ(result.return_waypoints.size(), 2u);
  EXPECT_NEAR(result.return_waypoints.front().yaw, 1.2, 1e-9);
  EXPECT_NEAR(result.return_waypoints.back().position.x(), 0.0, 1e-9);
}

TEST(MissionReturnPlanner, RejectsUnsafeOriginalChain) {
  const std::vector<MissionWaypoint> mission = {
      wp(0.0, 0.0, 0.0), wp(1.0, 0.0, 0.0), wp(2.0, 0.0, 0.0)};
  MissionReturnPlanner planner;
  const auto result = planner.plan(mission, mapPoint(1.0, 0.0, 0.0),
                                    MissionReturnConfig());
  EXPECT_EQ(result.status, MissionReturnStatus::NO_SAFE_PATH);
}
