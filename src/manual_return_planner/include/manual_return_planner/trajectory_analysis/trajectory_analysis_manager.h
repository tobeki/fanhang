#pragma once

#include "manual_return_planner/trajectory_analysis/trajectory_analysis_types.h"

#include <vector>

namespace manual_return_planner {

// Orchestrates the V2.0 trajectory-understanding pipeline on the FORWARD
// history (Home -> current position).  It runs the segmenter and the three
// detectors, assigns candidate ids, and re-labels segments.  It never deletes
// points and never changes the return path.
class TrajectoryAnalysisManager {
 public:
  TrajectoryAnalysisResult analyze(
      const std::vector<TrajectoryPoint>& forward_trajectory,
      const TrajectoryAnalysisConfig& config) const;
};

// The segment label that an edit candidate implies.  Exposed so that reporting
// code can pick the reason belonging to a segment's actual label instead of
// just the highest-confidence overlapping candidate.
SegmentType segmentTypeForEdit(EditType type);

}  // namespace manual_return_planner
