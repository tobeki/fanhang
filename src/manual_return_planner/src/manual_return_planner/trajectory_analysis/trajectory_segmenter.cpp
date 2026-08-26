#include "manual_return_planner/trajectory_analysis/trajectory_segmenter.h"

#include <cmath>

namespace manual_return_planner {
namespace {

double angleBetweenDeg(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  const double na = a.norm();
  const double nb = b.norm();
  if (na < 1e-9 || nb < 1e-9) return 0.0;
  double c = a.dot(b) / (na * nb);
  c = std::max(-1.0, std::min(1.0, c));
  return std::acos(c) * 180.0 / M_PI;
}

double speedAt(const std::vector<TrajectoryPoint>& p, std::size_t i) {
  const double vn = p[i].velocity.norm();
  if (vn > 1e-6) return vn;
  if (i == 0) return 0.0;
  const double dt = p[i].timestamp - p[i - 1].timestamp;
  if (dt <= 1e-9) return 0.0;
  return (p[i].position - p[i - 1].position).norm() / dt;
}

TrajectorySegment makeSegment(const std::vector<TrajectoryPoint>& points,
                              int start, int end, int id) {
  TrajectorySegment seg;
  seg.segment_id = id;
  seg.start_index = start;
  seg.end_index = end;
  seg.start_position = points[start].position;
  seg.end_position = points[end].position;
  seg.duration = points[end].timestamp - points[start].timestamp;
  double len = 0.0;
  for (int i = start + 1; i <= end; ++i)
    len += (points[i].position - points[i - 1].position).norm();
  seg.length = len;
  seg.average_speed = seg.duration > 1e-9 ? len / seg.duration : 0.0;
  Eigen::Vector3d dir = seg.end_position - seg.start_position;
  const double n = dir.norm();
  seg.direction_vector = n < 1e-9 ? Eigen::Vector3d::Zero()
                                 : Eigen::Vector3d(dir / n);
  return seg;
}

}  // namespace

std::vector<TrajectorySegment> TrajectorySegmenter::segment(
    const std::vector<TrajectoryPoint>& points,
    const TrajectoryAnalysisConfig& config) const {
  std::vector<TrajectorySegment> segments;
  const std::size_t n = points.size();
  if (n < 2) return segments;

  const double angle_threshold = config.segment_angle_threshold_deg;
  const double speed_change = config.segment_speed_change_threshold;

  auto dirAt = [&points](std::size_t i) {
    Eigen::Vector3d d = points[i + 1].position - points[i].position;
    const double len = d.norm();
    return len < 1e-9 ? Eigen::Vector3d::Zero() : Eigen::Vector3d(d / len);
  };

  int start = 0;
  int id = 0;
  for (std::size_t i = 1; i + 1 < n; ++i) {
    const double angle = angleBetweenDeg(dirAt(i - 1), dirAt(i));
    const double ds = std::abs(speedAt(points, i + 1) - speedAt(points, i));
    if (angle > angle_threshold || ds > speed_change) {
      segments.push_back(
          makeSegment(points, start, static_cast<int>(i), id++));
      start = static_cast<int>(i);
    }
  }
  segments.push_back(
      makeSegment(points, start, static_cast<int>(n - 1), id++));
  return segments;
}

}  // namespace manual_return_planner
