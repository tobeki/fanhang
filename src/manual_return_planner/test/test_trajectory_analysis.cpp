#include "manual_return_planner/manual_return_planner.h"
#include "manual_return_planner/trajectory_analysis/trajectory_analysis_manager.h"

#include <gtest/gtest.h>

#include <cstdlib>
#include <string>
#include <vector>

namespace {
using manual_return_planner::EditType;
using manual_return_planner::SegmentType;
using manual_return_planner::TrajectoryAnalysisConfig;
using manual_return_planner::TrajectoryAnalysisManager;
using manual_return_planner::TrajectoryAnalysisResult;
using manual_return_planner::TrajectoryPoint;

TrajectoryPoint pt(double t, double x, double y, double z = 0.0) {
  TrajectoryPoint p;
  p.timestamp = t;
  p.position << x, y, z;
  return p;
}

TrajectoryAnalysisResult analyze(const std::vector<TrajectoryPoint>& points) {
  TrajectoryAnalysisConfig config;
  TrajectoryAnalysisManager manager;
  return manager.analyze(points, config);
}
}  // namespace

// 1. A long straight flight must be classified NORMAL with no spurious
//    backtrack / overlap / dwell detections.
TEST(TrajectoryAnalysis, StraightLineIsNormal) {
  std::vector<TrajectoryPoint> pts;
  for (int i = 0; i <= 100; ++i)
    pts.push_back(pt(0.1 * i, 0.1 * i, 0.0));  // 1 m/s along x
  const auto r = analyze(pts);
  EXPECT_EQ(r.backtrack_count, 0);
  EXPECT_EQ(r.overlap_count, 0);
  for (const auto& s : r.segments)
    EXPECT_EQ(s.segment_type, SegmentType::NORMAL);
}

// 2. A dead-end spur A-B-C-B-D must be detected as BACKTRACK (covering B-C-B).
TEST(TrajectoryAnalysis, DetectsBacktrackSpur) {
  std::vector<TrajectoryPoint> pts;
  pts.push_back(pt(0.0, 0.0, 0.0));   // A
  pts.push_back(pt(5.0, 5.0, 0.0));   // B
  pts.push_back(pt(10.0, 5.0, 5.0));  // C (spur apex)
  pts.push_back(pt(15.0, 5.0, 0.0));  // B (return)
  pts.push_back(pt(20.0, 10.0, 0.0)); // D
  const auto r = analyze(pts);
  EXPECT_GE(r.backtrack_count, 1);
}

// 3. A repeated round-trip A-B-A-B must be detected as OVERLAP, not backtrack.
TEST(TrajectoryAnalysis, DetectsOverlapRoundTrip) {
  std::vector<TrajectoryPoint> pts;
  pts.push_back(pt(0.0, 0.0, 0.0));    // A
  pts.push_back(pt(10.0, 10.0, 0.0));  // B
  pts.push_back(pt(20.0, 0.0, 0.0));   // A
  pts.push_back(pt(30.0, 10.0, 0.0));  // B
  const auto r = analyze(pts);
  EXPECT_GE(r.overlap_count, 1);
  EXPECT_EQ(r.backtrack_count, 0);
}

// 4. A stationary hover (10 seconds at one position) must be detected as DWELL.
TEST(TrajectoryAnalysis, DetectsDwell) {
  std::vector<TrajectoryPoint> pts;
  for (int i = 0; i <= 100; ++i)
    pts.push_back(pt(0.1 * i, 1.0, 1.0, 1.0));  // stationary, 10 s
  const auto r = analyze(pts);
  EXPECT_GE(r.dwell_count, 1);
}

// ---------------------------------------------------------------------------
// Regression tests on a REAL recorded flight (2026-08-24).  These guard the
// exact failure that pure hand-built fixtures missed: on real data the
// detectors silently returned 0 backtrack and 0 overlap because the dead-end
// check rejected every genuine spur and the overlap threshold was ~3x too
// small.  The trajectory is:
//   (0,0) hover -> east to (10.3,1.3) -> turn back west to (3.1,3.8)
//   -> back east to (9.7,3.7) [dead-end spur]
//   -> south to (13.9,-7.1) hover -> (13.9,-9.1) hover
//   -> north back to (11.1,0.9) [reverse overlap with the southbound leg]
// Path length 52.6 m, straight-line start-to-end only 11.1 m.
namespace {

std::string realFlightCsvPath() {
#ifdef MRP_TEST_DATA_DIR
  const std::string dir(MRP_TEST_DATA_DIR);
#else
  const char* from_env = std::getenv("MRP_TEST_DATA_DIR");
  const std::string dir =
      from_env != nullptr ? std::string(from_env) : std::string("test/data");
#endif
  return dir + "/real_flight_backtrack_overlap.csv";
}

bool loadRealFlight(std::vector<TrajectoryPoint>* points) {
  std::string error;
  return manual_return_planner::CsvTrajectoryReader::read(realFlightCsvPath(),
                                                          points, &error);
}

}  // namespace

TEST(TrajectoryAnalysisRealFlight, DetectsDwellOnRealFlight) {
  std::vector<TrajectoryPoint> pts;
  ASSERT_TRUE(loadRealFlight(&pts)) << "cannot read " << realFlightCsvPath();
  ASSERT_GT(pts.size(), 600u);
  const auto r = analyze(pts);
  // Four hovers: after takeoff, at (13.9,-7.1), at (13.9,-9.1), before trigger.
  EXPECT_GE(r.dwell_count, 3);
}

TEST(TrajectoryAnalysisRealFlight, DetectsBacktrackOnRealFlight) {
  std::vector<TrajectoryPoint> pts;
  ASSERT_TRUE(loadRealFlight(&pts)) << "cannot read " << realFlightCsvPath();
  const auto r = analyze(pts);
  // The west-then-east excursion around index 160..340 is a dead-end spur.
  EXPECT_GE(r.backtrack_count, 1)
      << "real dead-end spur must not be rejected by the dead-end check";
}

TEST(TrajectoryAnalysisRealFlight, DetectsOverlapOnRealFlight) {
  std::vector<TrajectoryPoint> pts;
  ASSERT_TRUE(loadRealFlight(&pts)) << "cannot read " << realFlightCsvPath();
  const auto r = analyze(pts);
  // The southbound leg and the northbound return leg run alongside each other
  // with a ~1 m lateral offset.
  EXPECT_GE(r.overlap_count, 1)
      << "real reverse traversal must be within the calibrated threshold";
}

// A straight flight must stay clean even with the relaxed overlap threshold:
// two collinear segments that merely touch end-to-end are NOT a repeat.
TEST(TrajectoryAnalysis, RelaxedThresholdDoesNotCreateFalseOverlap) {
  std::vector<TrajectoryPoint> pts;
  for (int i = 0; i <= 300; ++i)
    pts.push_back(pt(0.1 * i, 0.1 * i, 0.0));  // 30 m straight line
  const auto r = analyze(pts);
  EXPECT_EQ(r.overlap_count, 0);
  EXPECT_EQ(r.backtrack_count, 0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
