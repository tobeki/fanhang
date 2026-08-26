#pragma once

#include "manual_return_planner/trajectory_analysis/trajectory_analysis_types.h"

#include <vector>

namespace manual_return_planner {

// Detects dead-end spurs (A-B-C-B-D -> B-C-B) with a geometric + time method.
// A window [i, j] is a backtrack candidate when:
//   1. distance(Pi, Pj) < epsilon_entry          (entry ~= exit)
//   2. detour ratio rho = path_len/direct > min_spur_ratio
//   3. max excursion from entry > min_spur_reach
//   4. entry and exit directions are opposite (dot < 0)
//   5. the spur interior is NOT revisited after the exit (dead-end check,
//      distinguishes a spur from a repeated round-trip A-B-A-B).
// Emits BACKTRACK candidates only (no deletion).
//
// A graph-based spur detector (leaf degree == 1) is intentionally deferred to
// a later stage; the interface keeps this class focused on geometry.
class BacktrackDetector {
 public:
  std::vector<TrajectoryEditCandidate> detect(
      const std::vector<TrajectoryPoint>& points,
      const TrajectoryAnalysisConfig& config) const;
};

}  // namespace manual_return_planner
