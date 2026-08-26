#include "manual_return_planner/trajectory_analysis/overlap_detector.h"

#include "manual_return_planner/manual_return_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace manual_return_planner {
namespace {

double pointToSegmentPolyline(const Eigen::Vector3d& p,
                             const std::vector<TrajectoryPoint>& points,
                             const TrajectorySegment& s) {
  double best = std::numeric_limits<double>::infinity();
  for (int k = s.start_index; k < s.end_index; ++k) {
    best = std::min(best, RdpSimplifier::pointToSegmentDistance(
                              p, points[k].position, points[k + 1].position));
  }
  return best;
}

// Fraction of points on segment `from` whose distance to segment `to` is below
// `threshold`, plus the mean of those distances.
void overlapStats(const std::vector<TrajectoryPoint>& points,
                  const TrajectorySegment& from, const TrajectorySegment& to,
                  double threshold, double* fraction, double* mean_distance) {
  int close = 0;
  int total = 0;
  double sum = 0.0;
  for (int k = from.start_index; k <= from.end_index; ++k) {
    const double d = pointToSegmentPolyline(points[k].position, points, to);
    ++total;
    sum += d;
    if (d < threshold) ++close;
  }
  *fraction = total > 0 ? static_cast<double>(close) / total : 0.0;
  *mean_distance = total > 0 ? sum / total
                             : std::numeric_limits<double>::infinity();
}

double angleBetweenDeg(const Eigen::Vector3d& u, const Eigen::Vector3d& v) {
  const double nu = u.norm();
  const double nv = v.norm();
  if (nu < 1e-9 || nv < 1e-9) return 0.0;
  double c = u.dot(v) / (nu * nv);
  c = std::max(-1.0, std::min(1.0, c));
  return std::acos(c) * 180.0 / M_PI;
}

}  // namespace

std::vector<TrajectoryEditCandidate> OverlapDetector::detect(
    const std::vector<TrajectoryPoint>& points,
    const std::vector<TrajectorySegment>& segments,
    const TrajectoryAnalysisConfig& config) const {
  std::vector<TrajectoryEditCandidate> out;
  const double angle_thr = config.overlap_angle_threshold_deg;
  const double dist_thr = config.overlap_distance_threshold;
  const double frac_thr = config.overlap_min_fraction;

  for (std::size_t a = 0; a < segments.size(); ++a) {
    if (segments[a].length < config.overlap_min_segment_length) continue;
    for (std::size_t b = a + 2; b < segments.size(); ++b) {  // non-adjacent
      if (segments[b].length < config.overlap_min_segment_length) continue;

      double frac_a = 0.0, mean_a = 0.0, frac_b = 0.0, mean_b = 0.0;
      overlapStats(points, segments[a], segments[b], dist_thr, &frac_a, &mean_a);
      overlapStats(points, segments[b], segments[a], dist_thr, &frac_b, &mean_b);
      // BOTH segments must largely run alongside the other, otherwise two
      // collinear segments touching end-to-end would be reported.
      if (frac_a < frac_thr || frac_b < frac_thr) continue;

      const double ang = angleBetweenDeg(segments[a].direction_vector,
                                         segments[b].direction_vector);
      TrajectoryEditCandidate c;
      c.type = EditType::OVERLAP;
      c.start_index = segments[a].start_index;
      c.end_index = segments[b].end_index;
      if (ang < angle_thr) {
        c.reason = "duplicate traversal (same direction)";
      } else if (ang > 180.0 - angle_thr) {
        c.reason = "reverse traversal (return along same region)";
      } else {
        continue;  // overlapping region but not aligned: not a repeat
      }
      const double mean_distance = std::max(mean_a, mean_b);
      const double min_fraction = std::min(frac_a, frac_b);
      c.metrics = "mean_distance=" + std::to_string(mean_distance) +
                  "m,overlap_fraction=" + std::to_string(min_fraction) +
                  ",angle=" + std::to_string(ang) + "deg";
      c.confidence = std::max(0.5, std::min(0.95, min_fraction));
      out.push_back(c);
    }
  }
  return out;
}

}  // namespace manual_return_planner
