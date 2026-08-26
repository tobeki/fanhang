#include "manual_return_planner/trajectory_analysis/trajectory_analysis_manager.h"

#include "manual_return_planner/trajectory_analysis/backtrack_detector.h"
#include "manual_return_planner/trajectory_analysis/dwell_detector.h"
#include "manual_return_planner/trajectory_analysis/overlap_detector.h"
#include "manual_return_planner/trajectory_analysis/trajectory_segmenter.h"

namespace manual_return_planner {

const char* segmentTypeToString(SegmentType type) {
  switch (type) {
    case SegmentType::DWELL:
      return "DWELL";
    case SegmentType::BACKTRACK_CANDIDATE:
      return "BACKTRACK_CANDIDATE";
    case SegmentType::OVERLAP_CANDIDATE:
      return "OVERLAP_CANDIDATE";
    default:
      return "NORMAL";
  }
}

const char* editTypeToString(EditType type) {
  switch (type) {
    case EditType::DWELL:
      return "DWELL";
    case EditType::BACKTRACK:
      return "BACKTRACK";
    case EditType::OVERLAP:
      return "OVERLAP";
  }
  return "UNKNOWN";
}

namespace {

int severity(SegmentType t) {
  switch (t) {
    case SegmentType::BACKTRACK_CANDIDATE:
      return 3;
    case SegmentType::OVERLAP_CANDIDATE:
      return 2;
    case SegmentType::DWELL:
      return 1;
    default:
      return 0;
  }
}

}  // namespace

SegmentType segmentTypeForEdit(EditType t) {
  switch (t) {
    case EditType::DWELL:
      return SegmentType::DWELL;
    case EditType::BACKTRACK:
      return SegmentType::BACKTRACK_CANDIDATE;
    case EditType::OVERLAP:
      return SegmentType::OVERLAP_CANDIDATE;
  }
  return SegmentType::NORMAL;
}

TrajectoryAnalysisResult TrajectoryAnalysisManager::analyze(
    const std::vector<TrajectoryPoint>& forward_trajectory,
    const TrajectoryAnalysisConfig& config) const {
  TrajectoryAnalysisResult result;
  if (forward_trajectory.size() < 2) return result;

  // 1. Segment the forward history.
  TrajectorySegmenter segmenter;
  result.segments = segmenter.segment(forward_trajectory, config);

  // 2. Dwell detection.
  DwellDetector dwell;
  std::vector<TrajectoryEditCandidate> dwell_candidates =
      dwell.detect(forward_trajectory, config);

  // 3. Backtrack (dead-end spur) detection.
  BacktrackDetector backtrack;
  std::vector<TrajectoryEditCandidate> backtrack_candidates =
      backtrack.detect(forward_trajectory, config);

  // 4. Overlap (repeated traversal) detection.
  OverlapDetector overlap;
  std::vector<TrajectoryEditCandidate> overlap_candidates =
      overlap.detect(forward_trajectory, result.segments, config);

  // 5. Assign ids and collect candidates.
  int next_id = 0;
  auto append = [&](std::vector<TrajectoryEditCandidate>& candidates) {
    for (auto& c : candidates) {
      c.candidate_id = next_id++;
      result.edit_candidates.push_back(c);
    }
  };
  append(dwell_candidates);
  append(backtrack_candidates);
  append(overlap_candidates);

  // 6. Re-label segments by intersecting candidates (higher severity wins).
  for (auto& seg : result.segments) {
    for (const auto& c : result.edit_candidates) {
      // A segment is labelled only when it is FULLY contained in the
      // candidate range (a flight segment that merely grazes a dwell
      // candidate must stay NORMAL).
      if (c.start_index <= seg.start_index && seg.end_index <= c.end_index) {
        const SegmentType t = segmentTypeForEdit(c.type);
        if (severity(t) > severity(seg.segment_type)) seg.segment_type = t;
      }
    }
  }

  result.dwell_count = static_cast<int>(dwell_candidates.size());
  result.backtrack_count = static_cast<int>(backtrack_candidates.size());
  result.overlap_count = static_cast<int>(overlap_candidates.size());
  return result;
}

}  // namespace manual_return_planner
