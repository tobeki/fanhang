#pragma once

#include "manual_return_planner/trajectory_analysis/trajectory_analysis_types.h"

#include <vector>

namespace manual_return_planner {

// Splits the forward trajectory into contiguous segments based on direction
// changes and sudden speed changes.  The output segments are all NORMAL; other
// detectors later re-label them.
class TrajectorySegmenter {
 public:
  std::vector<TrajectorySegment> segment(
      const std::vector<TrajectoryPoint>& points,
      const TrajectoryAnalysisConfig& config) const;
};

}  // namespace manual_return_planner
