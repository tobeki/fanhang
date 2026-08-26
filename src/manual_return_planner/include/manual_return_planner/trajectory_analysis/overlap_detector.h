#pragma once

#include "manual_return_planner/trajectory_analysis/trajectory_analysis_types.h"

#include <vector>

namespace manual_return_planner {

// Detects repeated traversal of a region: two non-adjacent segments whose
// spatial distance is below overlap_distance_threshold and whose directions are
// either nearly identical (duplicate traversal) or nearly opposite (return).
// Emits OVERLAP candidates only (no deletion).
class OverlapDetector {
 public:
  std::vector<TrajectoryEditCandidate> detect(
      const std::vector<TrajectoryPoint>& points,
      const std::vector<TrajectorySegment>& segments,
      const TrajectoryAnalysisConfig& config) const;
};

}  // namespace manual_return_planner
