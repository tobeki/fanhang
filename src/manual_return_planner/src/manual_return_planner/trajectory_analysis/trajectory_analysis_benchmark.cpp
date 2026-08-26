#include "manual_return_planner/manual_return_planner.h"
#include "manual_return_planner/trajectory_analysis/trajectory_analysis_manager.h"

#include <iostream>
#include <string>
#include <vector>

// Offline V2.0 trajectory-understanding benchmark.  Reads a forward-history CSV
// and prints the analysis summary.  Usage:
//   trajectory_analysis_benchmark <scenario> <trajectory_csv>
int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: trajectory_analysis_benchmark <scenario> <csv>\n";
    return 1;
  }
  const std::string scenario = argv[1];
  const std::string csv = argv[2];

  std::vector<manual_return_planner::TrajectoryPoint> points;
  std::string err;
  if (!manual_return_planner::CsvTrajectoryReader::read(csv, &points, &err)) {
    std::cerr << "read CSV failed: " << err << "\n";
    return 2;
  }

  manual_return_planner::TrajectoryAnalysisConfig config;
  manual_return_planner::TrajectoryAnalysisManager manager;
  const auto r = manager.analyze(points, config);

  std::cout << "scenario=" << scenario << " points=" << points.size()
            << " segments=" << r.segments.size() << " dwell=" << r.dwell_count
            << " backtrack=" << r.backtrack_count
            << " overlap=" << r.overlap_count << "\n";
  for (const auto& c : r.edit_candidates) {
    std::cout << "  cand" << c.candidate_id << " [" << c.start_index
              << "," << c.end_index << "] "
              << manual_return_planner::editTypeToString(c.type) << " "
              << c.reason << " conf=" << c.confidence << "\n";
  }
  for (const auto& s : r.segments) {
    std::cout << "  seg" << s.segment_id << " [" << s.start_index << ","
              << s.end_index << "] "
              << manual_return_planner::segmentTypeToString(s.segment_type)
              << " len=" << s.length << " dur=" << s.duration << "\n";
  }
  return 0;
}
