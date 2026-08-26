#pragma once

#include "manual_return_planner/manual_return_planner.h"

#include <Eigen/Core>

#include <string>
#include <vector>

namespace manual_return_planner {

// Classification of a contiguous span of the forward history trajectory.
enum class SegmentType {
  NORMAL,               // ordinary flight
  DWELL,                // hover / task stop
  BACKTRACK_CANDIDATE,  // dead-end spur (enter a region then return)
  OVERLAP_CANDIDATE     // repeated traversal of a region
};

const char* segmentTypeToString(SegmentType type);

struct TrajectorySegment {
  int segment_id = -1;
  int start_index = 0;  // inclusive index into the forward trajectory
  int end_index = 0;    // inclusive index into the forward trajectory
  double length = 0.0;        // [m]
  double duration = 0.0;      // [s]
  double average_speed = 0.0; // [m/s]
  Eigen::Vector3d start_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d end_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d direction_vector = Eigen::Vector3d::Zero();  // unit vector
  SegmentType segment_type = SegmentType::NORMAL;
};

enum class EditType {
  DWELL,
  BACKTRACK,
  OVERLAP
};

const char* editTypeToString(EditType type);

// A candidate edit.  V2.0 ONLY detects; it never deletes points or changes the
// return path.
struct TrajectoryEditCandidate {
  int candidate_id = -1;
  EditType type = EditType::BACKTRACK;
  int start_index = 0;
  int end_index = 0;
  double confidence = 0.0;  // [0, 1]
  std::string reason;
  std::string metrics;      // "key=value,..." evidence summary
};

struct TrajectoryAnalysisConfig {
  // Segmenter.
  double segment_angle_threshold_deg = 30.0;
  double segment_speed_change_threshold = 0.5;  // [m/s]

  // Dwell detection.
  double dwell_max_speed = 0.1;   // [m/s]
  double dwell_radius = 0.15;     // [m]
  double dwell_min_time = 2.0;    // [s]

  // Backtrack (dead-end spur) detection.
  double backtrack_epsilon_entry = 0.5;   // [m] entry ~= exit
  double backtrack_min_spur_ratio = 3.0;  // detour / direct distance
  double backtrack_min_spur_reach = 0.5;  // [m] max excursion from entry
  // Graph-based dead-end confirmation: the spur apex (the point furthest from
  // the entry) must not be revisited outside [i, j].  Checking only the apex
  // instead of every interior point is essential -- the interior near the
  // spur "neck" is inherently close to the outbound/return legs of the spur
  // itself, so an all-interior test rejects every real backtrack.
  double backtrack_apex_revisit_radius = 0.5;  // [m]

  // Overlap (repeated traversal) detection.
  //
  // overlap_distance_threshold is calibrated from real flight data: the
  // outbound and return legs are planned independently, so their lateral
  // offset is around 1 m rather than a few centimetres.
  //
  // overlap_min_fraction guards against a false positive that a pure distance
  // test cannot reject: two collinear segments that merely touch end-to-end
  // have a small average distance, but only a few points near the junction are
  // actually close.  Requiring a large FRACTION of points on BOTH segments to
  // be close means the two segments must genuinely run alongside each other.
  double overlap_distance_threshold = 1.2;    // [m]
  double overlap_angle_threshold_deg = 30.0;  // same-direction angle bound
  double overlap_min_segment_length = 0.5;    // [m] skip hover/tiny segments
  // Measured on the 2026-08-24 flight: the genuine reverse-traversal pair
  // reaches min(frac_a, frac_b) = 0.56 at a 1.2 m threshold, so 0.5 keeps
  // it while still demanding that half of BOTH segments run alongside.
  double overlap_min_fraction = 0.5;          // [0,1] both directions
};

struct TrajectoryAnalysisResult {
  std::vector<TrajectorySegment> segments;
  std::vector<TrajectoryEditCandidate> edit_candidates;
  int dwell_count = 0;
  int backtrack_count = 0;
  int overlap_count = 0;
};

}  // namespace manual_return_planner
