#include "manual_return_planner/manual_return_planner.h"
#include "manual_return_planner/safe_rdp.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <string>

// Offline Safe-RDP benchmark: loads a recorded flight CSV and a PCD map, runs
// the full planManualReturn pipeline (preprocess -> reverse -> RDP -> Safe-RDP),
// and prints the Safe-RDP metrics.  Usage:
//   safe_rdp_benchmark <scenario> <trajectory_csv> <pcd_path>
int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: safe_rdp_benchmark <scenario> <trajectory_csv> <pcd_path>\n";
    return 1;
  }
  const std::string scenario = argv[1];
  const std::string csv = argv[2];
  const std::string pcd = argv[3];

  std::vector<manual_return_planner::TrajectoryPoint> history;
  std::string err;
  if (!manual_return_planner::CsvTrajectoryReader::read(csv, &history, &err)) {
    std::cerr << "read CSV failed: " << err << "\n";
    return 2;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd, *cloud) != 0) {
    std::cerr << "load PCD failed: " << pcd << "\n";
    return 3;
  }

  manual_return_planner::ManualReturnConfig config;
  config.safe_rdp.enabled = true;

  manual_return_planner::ManualReturnPlanner planner;
  const auto r = planner.planManualReturn(history, cloud, config);

  std::cout << "scenario=" << scenario
            << " status=" << manual_return_planner::statusToString(r.status)
            << " original_points=" << r.raw_point_num
            << " simplified_points=" << r.final_point_num
            << " safe_rdp_enabled=" << (r.safe_rdp_enabled ? "true" : "false")
            << " clearance_available=" << (r.clearance_available ? "true" : "false")
            << " min_clearance_m=" << r.min_clearance_m
            << " collision_check_count=" << r.collision_check_count
            << " unsafe_segments=" << r.unsafe_segments
            << " validated_segments=" << r.validated_segments
            << " safe_path_points=" << r.safe_point_num
            << " original_rdp_points=" << r.original_rdp_point_num
            << " shortcut_count=" << r.shortcut_count
            << " voxelized_cloud_size=" << r.voxelized_cloud_size
            << " pointcloud_size=" << cloud->size()
            << "\n";

  return (r.status == manual_return_planner::ManualReturnStatus::NO_SAFE_PATH)
             ? 4
             : 0;
}
