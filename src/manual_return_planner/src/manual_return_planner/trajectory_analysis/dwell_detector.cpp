#include "manual_return_planner/trajectory_analysis/dwell_detector.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace manual_return_planner {
namespace {

double speedAt(const std::vector<TrajectoryPoint>& p, std::size_t i) {
  const double vn = p[i].velocity.norm();
  if (vn > 1e-6) return vn;
  if (i == 0) return 0.0;
  const double dt = p[i].timestamp - p[i - 1].timestamp;
  if (dt <= 1e-9) return 0.0;
  return (p[i].position - p[i - 1].position).norm() / dt;
}

}  // namespace

std::vector<TrajectoryEditCandidate> DwellDetector::detect(
    const std::vector<TrajectoryPoint>& points,
    const TrajectoryAnalysisConfig& config) const {
  std::vector<TrajectoryEditCandidate> out;
  const std::size_t n = points.size();
  if (n < 2) return out;

  std::vector<double> speed(n, 0.0);
  for (std::size_t i = 0; i < n; ++i) speed[i] = speedAt(points, i);

  // Find maximal runs of consecutive "slow" points, then require each run to
  // be long enough in time and small enough in space.  This avoids the
  // takeoff-deceleration / hover / departure-acceleration from being merged
  // into one over-long candidate (unlike a distance-from-window-start test).
  std::size_t i = 0;
  while (i < n) {
    if (speed[i] >= config.dwell_max_speed) {
      ++i;
      continue;
    }
    std::size_t j = i;
    while (j + 1 < n && speed[j + 1] < config.dwell_max_speed) ++j;

    const double dur = points[j].timestamp - points[i].timestamp;
    double max_dist = 0.0;
    for (std::size_t a = i; a <= j; ++a)
      for (std::size_t b = a + 1; b <= j; ++b)
        max_dist = std::max(
            max_dist, (points[a].position - points[b].position).norm());

    if (dur >= config.dwell_min_time && max_dist < config.dwell_radius) {
      double avg_speed = 0.0;
      for (std::size_t k = i; k <= j; ++k) avg_speed += speed[k];
      avg_speed /= static_cast<double>(j - i + 1);

      TrajectoryEditCandidate c;
      c.type = EditType::DWELL;
      c.start_index = static_cast<int>(i);
      c.end_index = static_cast<int>(j);
      c.reason = "dwell/hover: low speed in small region";
      c.metrics = "duration=" + std::to_string(dur) +
                  "s,avg_speed=" + std::to_string(avg_speed) +
                  "m/s,extent=" + std::to_string(max_dist) + "m";
      const double speed_margin = 1.0 - avg_speed / config.dwell_max_speed;
      c.confidence =
          std::max(0.5, std::min(0.95, 0.6 + 0.4 * speed_margin));
      out.push_back(c);
    }
    i = j + 1;
  }
  return out;
}

}  // namespace manual_return_planner
