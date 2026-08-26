#pragma once

#include "manual_return_planner/trajectory_analysis/trajectory_analysis_types.h"

#include <vector>

namespace manual_return_planner {

// Detects hover / task-stop intervals with a sliding window: a window whose
// duration >= dwell_min_time, average speed < dwell_max_speed, and spatial
// extent < dwell_radius.  Emits DWELL candidates only (no deletion).
class DwellDetector {
 public:
  std::vector<TrajectoryEditCandidate> detect(
      const std::vector<TrajectoryPoint>& points,
      const TrajectoryAnalysisConfig& config) const;
};

}  // namespace manual_return_planner
