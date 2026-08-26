#include "manual_return_planner/trajectory_analysis/backtrack_detector.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace manual_return_planner {
namespace {

// Index of the point in [i, j] that is furthest from the entry point.  That
// point is the spur apex: for a dead-end excursion it is the "tip" that is
// visited exactly once.
std::size_t apexIndex(const std::vector<TrajectoryPoint>& points,
                      std::size_t i, std::size_t j, double* reach_out) {
  std::size_t apex = i;
  double reach = 0.0;
  for (std::size_t k = i; k <= j; ++k) {
    const double dist = (points[k].position - points[i].position).norm();
    if (dist > reach) {
      reach = dist;
      apex = k;
    }
  }
  if (reach_out) *reach_out = reach;
  return apex;
}

// Dead-end confirmation: the apex must NOT be visited again outside [i, j].
// A repeated round-trip (A-B-A-B) revisits its apex, a true dead-end spur does
// not.  Only the apex is tested -- the interior near the spur neck is
// inherently close to the spur's own legs, so testing every interior point
// would reject every genuine backtrack.
bool apexRevisitedOutside(const std::vector<TrajectoryPoint>& points,
                          std::size_t i, std::size_t j, std::size_t apex,
                          double radius) {
  const Eigen::Vector3d& a = points[apex].position;
  for (std::size_t k = 0; k < points.size(); ++k) {
    if (k >= i && k <= j) continue;  // skip the spur itself
    if ((points[k].position - a).norm() < radius) return true;
  }
  return false;
}

}  // namespace

std::vector<TrajectoryEditCandidate> BacktrackDetector::detect(
    const std::vector<TrajectoryPoint>& points,
    const TrajectoryAnalysisConfig& config) const {
  std::vector<TrajectoryEditCandidate> out;
  const std::size_t n = points.size();
  if (n < 3) return out;

  // Prefix path length so the detour ratio is O(1) per candidate instead of
  // O(n); the scan below is otherwise O(n^2) on 10 Hz histories.
  std::vector<double> prefix(n, 0.0);
  for (std::size_t k = 1; k < n; ++k)
    prefix[k] = prefix[k - 1] +
                (points[k].position - points[k - 1].position).norm();

  std::size_t i = 0;
  while (i + 2 < n) {
    // Prefer the LARGEST spur starting at i: scan j downwards so that a long
    // excursion is reported as one candidate instead of many nested ones.
    bool matched = false;
    for (std::size_t j = n - 1; j >= i + 2; --j) {
      const double direct = (points[j].position - points[i].position).norm();
      if (direct > config.backtrack_epsilon_entry) continue;  // entry ~= exit

      const double path_len = prefix[j] - prefix[i];
      const double rho = path_len / std::max(direct, 1e-3);
      if (rho <= config.backtrack_min_spur_ratio) continue;

      double reach = 0.0;
      const std::size_t apex = apexIndex(points, i, j, &reach);
      if (reach <= config.backtrack_min_spur_reach) continue;

      // Entry and exit directions must be opposite.
      const Eigen::Vector3d v_in = points[i + 1].position - points[i].position;
      const Eigen::Vector3d v_out = points[j].position - points[j - 1].position;
      const double n_in = v_in.norm();
      const double n_out = v_out.norm();
      if (n_in < 1e-9 || n_out < 1e-9) continue;
      if (v_in.dot(v_out) / (n_in * n_out) >= 0.0) continue;

      // Dead-end confirmation on the apex only.
      if (apexRevisitedOutside(points, i, j, apex,
                               config.backtrack_apex_revisit_radius)) {
        continue;
      }

      TrajectoryEditCandidate c;
      c.type = EditType::BACKTRACK;
      c.start_index = static_cast<int>(i);
      c.end_index = static_cast<int>(j);
      c.reason = "geometric backtrack candidate (dead-end spur: enter, reach "
                 "apex, return to entry)";
      c.metrics = "entry_dist=" + std::to_string(direct) +
                  "m,rho=" + std::to_string(rho) +
                  ",reach=" + std::to_string(reach) +
                  "m,apex_index=" + std::to_string(apex);
      const double rho_margin =
          std::min(1.0, rho / (config.backtrack_min_spur_ratio * 2.0));
      const double reach_margin =
          std::min(1.0, reach / (config.backtrack_min_spur_reach * 2.0));
      c.confidence = std::max(
          0.5, std::min(0.95, 0.5 + 0.5 * std::min(rho_margin, reach_margin)));
      out.push_back(c);
      matched = true;
      i = j + 1;
      break;
    }
    if (!matched) ++i;
  }
  return out;
}

}  // namespace manual_return_planner
